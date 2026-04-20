import asyncio
from dataclasses import dataclass
import re
import time

from pysnmp.hlapi.v3arch.asyncio import (
    CommunityData,
    ContextData,
    ObjectIdentity,
    ObjectType,
    SnmpEngine,
    UdpTransportTarget,
    get_cmd,
    next_cmd,
)


DEFAULT_COMMUNITY = "public"
IF_NAME_OID = "1.3.6.1.2.1.31.1.1.1.1"
IF_DESCR_OID = "1.3.6.1.2.1.2.2.1.2"
IF_HC_IN_OCTETS_OID = "1.3.6.1.2.1.31.1.1.1.6"
IF_HC_OUT_OCTETS_OID = "1.3.6.1.2.1.31.1.1.1.10"

PORT_RE = re.compile(r"\bPort:\s*([0-9]+)\b")


@dataclass(frozen=True)
class Interface:
    index: int
    name: str
    descr: str


@dataclass(frozen=True)
class Sample:
    captured_at: float
    in_octets: int
    out_octets: int


class SnmpClient:
    def __init__(self, host: str, community: str) -> None:
        self.engine = SnmpEngine()
        self.auth = CommunityData(community)
        self.context = ContextData()
        self.host = host
        self.target: UdpTransportTarget | None = None

    async def connect(self) -> None:
        self.target = await UdpTransportTarget.create((self.host, 161))

    async def get_value(self, oid: str) -> str:
        assert self.target is not None
        error_indication, error_status, error_index, var_binds = await get_cmd(
            self.engine,
            self.auth,
            self.target,
            self.context,
            ObjectType(ObjectIdentity(oid)),
            lookupMib=False,
        )
        self._raise_on_error(error_indication, error_status, error_index)
        return var_binds[0][1].prettyPrint()

    async def walk(self, base_oid: str) -> dict[int, str]:
        assert self.target is not None
        results: dict[int, str] = {}
        current_oid = base_oid

        while True:
            error_indication, error_status, error_index, var_binds = await next_cmd(
                self.engine,
                self.auth,
                self.target,
                self.context,
                ObjectType(ObjectIdentity(current_oid)),
                lookupMib=False,
            )
            self._raise_on_error(error_indication, error_status, error_index)

            returned_oid, returned_value = var_binds[0]
            oid_text = returned_oid.prettyPrint()
            if not oid_text.startswith(base_oid + "."):
                return results

            results[int(oid_text.rsplit(".", 1)[1])] = returned_value.prettyPrint()
            current_oid = oid_text

    async def discover_interface(self, switch_port_number: int) -> Interface:
        names, descrs = await asyncio.gather(
            self.walk(IF_NAME_OID),
            self.walk(IF_DESCR_OID),
        )

        expected_name = f"0/{switch_port_number}"
        for if_index, name in names.items():
            descr = descrs.get(if_index, "")
            if name == expected_name:
                return Interface(index=if_index, name=name, descr=descr)

        for if_index, descr in descrs.items():
            match = PORT_RE.search(descr)
            if match and int(match.group(1)) == switch_port_number:
                return Interface(index=if_index, name=names.get(if_index, ""), descr=descr)

        raise RuntimeError(f"could not find SNMP interface for physical switch port {switch_port_number}")

    async def read_sample(self, if_index: int) -> Sample:
        in_octets, out_octets = await asyncio.gather(
            self.get_value(f"{IF_HC_IN_OCTETS_OID}.{if_index}"),
            self.get_value(f"{IF_HC_OUT_OCTETS_OID}.{if_index}"),
        )
        return Sample(
            captured_at=time.monotonic(),
            in_octets=int(in_octets),
            out_octets=int(out_octets),
        )

    @staticmethod
    def out_mbps(previous: Sample, current: Sample) -> float | None:
        elapsed = current.captured_at - previous.captured_at
        if elapsed <= 0:
            return None

        delta_out = current.out_octets - previous.out_octets
        if delta_out < 0:
            return None

        return (delta_out * 8 / elapsed) / 1_000_000

    @staticmethod
    def _raise_on_error(error_indication, error_status, error_index) -> None:
        if error_indication:
            raise RuntimeError(str(error_indication))
        if error_status:
            raise RuntimeError(f"SNMP error at index {error_index}: {error_status.prettyPrint()}")

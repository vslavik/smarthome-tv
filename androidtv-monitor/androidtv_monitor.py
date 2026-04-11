import logging
from typing import Any, Literal

from androidtv import setup as androidtv_setup
import msgspec


logger = logging.getLogger(__name__)


# Names of common apps, particularly ones that need special handling:
LAUNCHER_APP = "com.google.android.tvlauncher"
SETTINGS_APP = "com.android.tv.settings"
NETFLIX_APP = "com.netflix.ninja"
PLEX_APP = "com.plexapp.android"
YOUTUBE_APP = "com.google.android.youtube.tv"
APPLE_TV_APP = "com.apple.atve.androidtv.appletv"


class AndroidTVState(msgspec.Struct):
    """Observed playback state for the Android TV device.

    Possible `state` values:
    - `idle`: app open or launcher/settings visible, but not actively playback
    - `playback`: active playback detected
    - `paused`: playback paused but still resumable
    - `unavailable`: probe failed or ADB connection is down
    """

    state: Literal["idle", "playback", "paused", "unavailable"] | None
    current_app: str | None
    media_session_state: int | None
    audio_state: str | None
    error: str | None = None

    def change_key(self) -> tuple[object, ...]:
        return (
            self.state,
            self.current_app,
            self.media_session_state,
            self.audio_state,
            self.error,
        )


def state_from_media_session(media_session_state: int | None) -> str:
    if media_session_state == 2:
        return "paused"
    if media_session_state == 3:
        return "playback"
    return "idle"


def derive_state(
    current_app: str | None,
    media_session_state: int | None,
    audio_state: str | None,
) -> str:
    """
    Given reported Android playback state, apply app-specific fixups and return
    a normalized state string. This is necessary because the raw reported state is often
    ambiguous or incorrect for certain apps.
    """
    if current_app in (None, SETTINGS_APP, LAUNCHER_APP):
        return "idle"

    if current_app == PLEX_APP:
        return state_from_media_session(media_session_state)

    if current_app == NETFLIX_APP:
        return state_from_media_session(media_session_state)

    if current_app == YOUTUBE_APP:
        return state_from_media_session(media_session_state)

    if current_app == APPLE_TV_APP:
        if media_session_state == 2 and audio_state == "idle":
            return "idle"
        return state_from_media_session(media_session_state)

    if media_session_state:
        return state_from_media_session(media_session_state)

    if audio_state and audio_state != "idle":
        return audio_state

    return "idle"


class AndroidTVMonitor:
    def __init__(self, *, host: str, port: int, adbkey: str) -> None:
        self.host = host
        self.port = port
        self.adbkey = adbkey
        self._device: Any | None = None

    def close(self) -> None:
        try:
            if self._device is not None:
                self._device.adb_close()
        finally:
            self._device = None

    def connect(self) -> None:
        """
        Establish an ADB connection to the device. This is called automatically by poll().
        """
        self._device = androidtv_setup(
            host=self.host,
            port=self.port,
            adbkey=self.adbkey,
            device_class="androidtv",
            log_errors=False,
        )

        if not self._device.available:
            self.close()
            raise RuntimeError(
                "ADB connection failed. Make sure ADB debugging is enabled on the "
                "Shield, the device has authorized this adb key, and the host/port "
                "are correct."
            )

    def poll(self) -> AndroidTVState:
        """
        Poll the device for its current state.
        """
        try:
            if self._device is None or not self._device.available:
                self.close()
                self.connect()

            assert self._device is not None
            current_app, media_session_state = self._device.current_app_media_session_state()
            audio_state = self._device.audio_state()
            return AndroidTVState(
                state=derive_state(current_app, media_session_state, audio_state),
                current_app=current_app,
                media_session_state=media_session_state,
                audio_state=audio_state,
            )
        except Exception as exc:  # noqa: BLE001
            logger.exception("failed to poll Android TV state")
            self.close()
            return AndroidTVState(
                state="unavailable",
                current_app=None,
                media_session_state=None,
                audio_state=None,
                error=str(exc),
            )

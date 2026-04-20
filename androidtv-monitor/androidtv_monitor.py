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

# Android PlaybackState constants:
# https://developer.android.com/reference/android/media/session/PlaybackState
MEDIA_SESSION_NONE = 0
MEDIA_SESSION_STOPPED = 1
MEDIA_SESSION_PAUSED = 2
MEDIA_SESSION_PLAYING = 3
MEDIA_SESSION_FAST_FORWARDING = 4
MEDIA_SESSION_REWINDING = 5
MEDIA_SESSION_BUFFERING = 6
MEDIA_SESSION_ERROR = 7
MEDIA_SESSION_CONNECTING = 8
MEDIA_SESSION_SKIPPING_TO_PREVIOUS = 9
MEDIA_SESSION_SKIPPING_TO_NEXT = 10
MEDIA_SESSION_SKIPPING_TO_QUEUE_ITEM = 11


class AndroidTVState(msgspec.Struct, omit_defaults=True):
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


def state_from_media_session(media_session_state: int | None) -> str | None:
    if media_session_state == MEDIA_SESSION_PAUSED:
        return "paused"
    if media_session_state == MEDIA_SESSION_PLAYING:
        return "playback"
    return None


def derive_state(
    current_app: str | None,
    media_session_state: int | None,
    audio_state: str | None,
) -> Literal["idle", "playback", "paused"] | None:
    """
    Given reported Android playback state, apply app-specific fixups and return
    a normalized state string. Return None for ambiguous samples that should not
    be published.
    """
    if current_app in (None, SETTINGS_APP, LAUNCHER_APP):
        return "idle"

    if current_app == APPLE_TV_APP:
        if media_session_state == MEDIA_SESSION_PAUSED and audio_state == "idle":
            return "idle"

    state = state_from_media_session(media_session_state)
    if state:
        return state

    if not media_session_state and audio_state == "idle":
        return "idle"

    return None


class AndroidTVMonitor:
    def __init__(self, *, host: str, port: int, adbkey: str) -> None:
        self.host = host
        self.port = port
        self.adbkey = adbkey
        self._device: Any | None = None
        self._last_state = "idle"

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
        self._last_state = "idle"

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

            current_state = derive_state(current_app, media_session_state, audio_state)
            if current_state:
                self._last_state = current_state
            else:
                current_state = self._last_state

            return AndroidTVState(
                state=current_state,
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

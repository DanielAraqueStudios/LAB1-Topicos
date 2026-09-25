"""User-facing wording for errors. Technical details go to the serial log only."""
from __future__ import annotations

MESSAGES = {
    "port_busy": ("Port is busy",
                  "Another program (Arduino IDE, serial monitor, PuTTY) is using this port. Close it and try again."),
    "port_missing": ("Port not found",
                     "The selected port is no longer available. Check the USB cable and press Refresh."),
    "open_failed": ("Could not open the port",
                    "Check the cable, the selected port and the baud rate, then try again."),
    "lost": ("Connection lost",
             "The device stopped responding or was unplugged. Reconnect the ESP32 and connect again."),
    "no_response": ("Device not responding",
                    "No answer from the ESP32. Check that the characterization firmware is flashed, "
                    "the baud rate is 115200, and press the EN button on the board."),
    "timeout": ("Transfer timed out",
                "The device stopped sending data before the run finished. Try the test again."),
    "malformed": ("Unreadable data",
                  "The data received could not be parsed. Try the test again; if it persists, check the cable."),
    "save_failed": ("Could not save the file",
                    "Check that the folder exists and that you have permission to write to it."),
    "load_failed": ("Could not open the file",
                    "The file is not a valid characterization CSV (expected header "
                    "k,t_us,u_pct,y_jib_raw,y_cable_raw)."),
}


def friendly(code: str) -> tuple[str, str]:
    return MESSAGES.get(code, ("Something went wrong", "Please try again."))

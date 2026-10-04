# Contributing

Bug reports and focused improvements are welcome. Before opening an issue,
check whether it has already been reported.

For a bug report, include your operating system, Python version, DJI camera
model, ExifTool version, and the command you ran. Include the error text and
whether `ffprobe` or FFmpeg is installed when relevant.

Do not attach original drone footage, telemetry dumps, or generated diagnostics
that could reveal private locations or other personal information. A short,
sanitized description is usually enough to start investigating.

For code changes, keep the command-line workflow focused and document any new
options or system requirements in the README. This MVP has no Python runtime
dependencies; changes should preserve that unless there is a clear need.

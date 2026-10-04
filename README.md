# DJI Cinematic Segment Cutter

Find smooth-looking segments in original DJI drone MP4s using the attitude
telemetry embedded in the video. The detector reads telemetry with ExifTool and
does not decode video frames.

The first release focuses on DJI Mini 4 Pro footage. It is a command-line tool;
there is no graphical interface yet.

## Requirements

- Python 3.10 or newer
- [ExifTool](https://exiftool.org/) to read embedded DJI telemetry
- [FFmpeg](https://ffmpeg.org/) for `ffprobe` frame-rate detection and direct
  segment export. Both commands are optional: without them, the detector uses
  its fallback frame rate and can still write reports and LosslessCut CSVs.
- [LosslessCut](https://github.com/mifi/lossless-cut) only if you want to open
  the generated edit-decision CSV and export segments from its interface

ExifTool and FFmpeg are separate system programs, not Python packages. Install
them with your operating system's package manager or from their official
downloads, and make sure their commands are available on your `PATH`.

## Install

Clone the repository, create a virtual environment, and install the CLI:

```bash
git clone https://github.com/hardik-id/dji-cinematic-segment-cutter.git
cd dji-cinematic-segment-cutter
python -m venv .venv
```

Activate the environment, then install the project:

```bash
# macOS / Linux
. .venv/bin/activate
python -m pip install .
```

On Windows PowerShell, activate with:

```powershell
.\.venv\Scripts\Activate.ps1
python -m pip install .
```

## Use

Analyze one video and print smooth segments:

```bash
dji-cinematic-segment-cutter /path/to/DJI_0001.MP4
```

Write machine-readable JSON, per-sample diagnostics, and a LosslessCut CSV:

```bash
dji-cinematic-segment-cutter /path/to/DJI_0001.MP4 \
  --json output/result.json \
  --debug-csv output/diagnostics.csv \
  --losslesscut-csv output/segments.csv
```

Open the source video in LosslessCut, choose **File → Import project**, and
select the generated CSV. LosslessCut stream-copies segments around nearby
keyframes; inspect the boundaries when frame-exact edits matter.

To analyze all MP4 files in a directory, add `--recursive` to include
subdirectories:

```bash
dji-cinematic-segment-cutter /path/to/videos --recursive --json output/
```

To write each detected segment directly as a separate MP4 without re-encoding,
install FFmpeg and use:

```bash
dji-cinematic-segment-cutter /path/to/DJI_0001.MP4 --cut output/cuts/
```

Direct cuts include the primary video, audio, and subtitle streams. FFmpeg
cannot retain DJI's proprietary telemetry tracks in these outputs. Stream-copy
cuts can begin on a nearby keyframe, so they are not frame-exact. Existing
output files are never overwritten.

## Tune detection

Pass a JSON config file or override individual settings on the command line.
For example:

```bash
dji-cinematic-segment-cutter /path/to/DJI_0001.MP4 \
  --min-duration 6 --jerk-sensitivity 1.3
```

Config file example:

```json
{
  "window_seconds": 0.75,
  "start_threshold": 85,
  "end_threshold": 70,
  "good_persistence": 0.75,
  "bad_persistence": 0.2,
  "min_duration": 4,
  "jerk_sensitivity": 1.0
}
```

Save it as `settings.json` and pass `--config settings.json`.

## Privacy and limitations

The JSON report records the video filename, not its full local path. Keep your
original footage and generated output local; they are excluded by `.gitignore`.
Only original DJI MP4s with readable embedded attitude telemetry are
supported. Detection is based on gimbal and aircraft movement, not image
content, so review suggested segments before using them in a final edit.

## License

MIT. See [LICENSE](LICENSE).

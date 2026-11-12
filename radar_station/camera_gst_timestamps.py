"""Copy MJPEG into AVI and timestamp each real jpegparse output buffer."""
import csv
import json
import time
from pathlib import Path

def atomic_json(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2), encoding='utf-8')
    temp.replace(path)

def record_camera(args, summary, should_stop):
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    Gst.init(None)
    if args.fourcc.upper() != 'MJPG' or args.save_every_n != 1:
        raise ValueError('Timestamped direct capture requires MJPG and save-every-n=1')
    device = '/dev/video' + args.device if args.device.isdigit() else args.device
    pipeline = Gst.parse_launch(
        f'v4l2src device={json.dumps(device)} io-mode=2 ! '
        f'image/jpeg,width={args.width},height={args.height},framerate={args.fps}/1 ! '
        f'jpegparse name=frames ! avimux ! filesink location={json.dumps(str(args.video_path))}'
    )
    summary.update(backend='gstreamer-copy-pad-timestamps', device=device,
        resolved_device=str(Path(device).resolve()), actual_width=args.width,
        actual_height=args.height, actual_fps=float(args.fps), actual_fourcc='MJPG',
        timestamp_strategy='actual_jpegparse_buffer_receive_time_and_gst_pts',
        timestamp_warning='Host timestamps are buffer receive times, not calibrated exposure times; Gst PTS is a separate pipeline clock.')
    count = 0
    first = last = None
    callback_errors = []
    eos_received = False
    error = None
    fields = ['frame_index', 'host_wall_time_ns', 'host_monotonic_ns', 'width', 'height',
              'gst_pts_ns', 'gst_dts_ns', 'gst_duration_ns', 'buffer_bytes']
    with args.csv_path.open('x', newline='', encoding='utf-8') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(fields)
        def on_buffer(_pad, info):
            nonlocal count, first, last
            try:
                buffer = info.get_buffer()
                if buffer is None:
                    return Gst.PadProbeReturn.OK
                wall, mono = time.time_ns(), time.monotonic_ns()
                stamp = lambda value: '' if value == Gst.CLOCK_TIME_NONE else int(value)
                writer.writerow([count + 1, wall, mono, args.width, args.height,
                    stamp(buffer.pts), stamp(buffer.dts), stamp(buffer.duration), buffer.get_size()])
                count += 1
                last = (wall, mono)
                if first is None:
                    first = last
                    csv_file.flush()
                    if args.ready_path:
                        atomic_json(args.ready_path, dict(status='ready', device=device,
                            backend=summary['backend'], first_frame_wall_time_ns=wall,
                            first_frame_monotonic_ns=mono, gst_pts_ns=stamp(buffer.pts)))
                elif count % max(1, args.fps // 5) == 0:
                    csv_file.flush()
            except Exception as exc:
                callback_errors.append(str(exc))
            return Gst.PadProbeReturn.OK
        pad = pipeline.get_by_name('frames').get_static_pad('src')
        pad.add_probe(Gst.PadProbeType.BUFFER, on_buffer)
        bus = pipeline.get_bus()
        try:
            if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError('GStreamer failed to start')
            summary['opened'] = True
            while not should_stop() and not callback_errors:
                message = bus.timed_pop_filtered(100 * Gst.MSECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if message:
                    if message.type == Gst.MessageType.ERROR:
                        issue, debug = message.parse_error()
                        error = str(issue) + ': ' + str(debug)
                    else:
                        eos_received = True
                    break
            if not eos_received and error is None:
                pipeline.send_event(Gst.Event.new_eos())
                message = bus.timed_pop_filtered(6 * Gst.SECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS)
                eos_received = bool(message and message.type == Gst.MessageType.EOS)
                if not eos_received:
                    error = 'AVI finalization did not confirm EOS'
        finally:
            pipeline.set_state(Gst.State.NULL)
        csv_file.flush()
    if callback_errors:
        error = '; '.join(callback_errors)
    duration = last[1] - first[1] if first and last else 0
    summary.update(saved_frames=count,
        first_frame_wall_time_ns=first[0] if first else None,
        last_frame_wall_time_ns=last[0] if last else None,
        first_frame_monotonic_ns=first[1] if first else None,
        first_saved_monotonic_ns=first[1] if first else None,
        last_saved_monotonic_ns=last[1] if last else None,
        saved_duration_ns=duration, actual_saved_fps=(count-1)*1e9/duration if duration else None,
        capture_stop_wall_time_ns=time.time_ns(), capture_stop_monotonic_ns=time.monotonic_ns(),
        eos_finalized=eos_received, error=error or (None if count else 'No camera frames received'))
    return 0 if count and not summary['error'] else 2

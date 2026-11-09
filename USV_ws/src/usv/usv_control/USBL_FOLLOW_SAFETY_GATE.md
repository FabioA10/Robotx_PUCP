# USBL follow: integration and actuation safety gate

This document defines the boundary between the current USBL follow preview and
any future command sent to the USV. It is an acceptance contract, not an
authorization to connect the preview node to MAVROS.

The supported hardware arrangement is an X150 mounted on the USV and an X110
or X010 on the UUV. Until the decoder and mounting transform are verified with
real captures, `/usbl/relative_pose` must come only from the simulator or from
reviewed, normalized replay data.

## Decoder boundary

The real adapter must keep the following stages separate:

1. **Transport capture** receives complete SeaTrac frames and records their
   original bytes, arrival time and transport. The diagnostic
   `seatrac/frame_raw` topic is evidence only; its current ASCII payload has
   not yet been classified or validated as a position fix.
2. **Protocol decoder** accepts only documented and fixture-tested CIDs. It
   validates framing, payload length, checksum/CRC when supplied by the
   protocol, validity flags and numeric ranges. Unknown or incomplete frames
   are diagnostics, never positions.
3. **Native measurement** preserves the device fields and their units without
   calling them forward/left/up. It also preserves beacon ID, acquisition time,
   range, fit error, RSSI and validity/filter flags when present.
4. **Frame normalizer** applies the measured X150 mounting transform, native
   axis convention and USV attitude needed at the acquisition instant. Its
   output is `base_link`, with `x` forward, `y` left and `z` up. A missing
   transform or attitude rejects the measurement instead of guessing it.
5. **Validator and publisher** apply monotonic-time, beacon, range, quality,
   covariance and jump gates before publishing
   `geometry_msgs/PoseWithCovarianceStamped` on `/usbl/relative_pose`.

The adapter must not infer UUV orientation from a single relative USBL
position. It must not silently substitute Easting/Northing/Depth for body axes.
Each captured CID used by the decoder needs both a raw-frame fixture and an
expected decoded result before it can be enabled.

## Time and QoS contract

`header.stamp` represents measurement acquisition time in the active ROS clock
domain. A device uptime counter cannot be copied directly into a ROS timestamp:
the adapter must estimate and monitor its offset, or explicitly use the time at
which the complete fix was received when no acquisition clock exists. The
chosen basis and observed transport latency must be published in diagnostics.

Clock jumps, a changed sample with a repeated timestamp, future stamps and
out-of-order samples are rejected. Bag replay of recorded timestamps requires
`use_sim_time` and `/clock`; live replay may restamp at publication time but
must be labelled as replay in diagnostics.

For the live relative-pose stream, publisher and subscriber must use a
compatible volatile sensor-data profile. A small depth avoids processing a
backlog as if it were current; freshness is still decided from the source
timestamp, never queue arrival. Before a hardware test, verify compatibility
with:

```bash
ros2 topic info -v /usbl/relative_pose
```

Raw capture has a different purpose: choose enough queue depth for the
observed frame rate and confirm bag loss statistics. QoS must not be used to
make old position samples appear valid.

## X150 capture plan

Do not run the fake publisher and the real driver in the same ROS graph. Leave
MAVROS and every USV velocity controller stopped during capture.

Record, under one test identifier:

- `/seatrac/frame_raw`, `/seatrac/status_raw`,
  `/seatrac/system_info_raw` and `/diagnostics`;
- host time, firmware, transport settings, X150 and remote model/ID;
- X150 pose and lever arm relative to `base_link`;
- water type, both transducer depths and tether configuration;
- externally measured UUV position for stationary centre, forward, aft, port
  and starboard cases;
- repeated stationary samples for at least 60 seconds at each usable range;
- deliberate signal obstruction and recovery, without moving either vehicle.

Retain the original bag and a checksum before converting anything to CSV. The
conversion must be reproducible and document every sign, unit and rotation.
For each test calculate update rate, median and maximum gap, latency when
measurable, rejection reasons, radial dispersion, 95th-percentile error and
the fraction of missing fixes. Pool multipath is part of the result, not an
outlier to remove without a stated rule.

The initial dataset is a **No-Go** if any direction has the wrong sign, the
normal maximum gap is at least the configured fix timeout, the 95th-percentile
stationary radial error reaches the 0.20 m deadband, timestamps cannot be put
in the ROS clock domain, or the capture cannot be replayed deterministically.

## Required command arbiter and gate

No follow component may publish directly to a MAVROS setpoint topic. Before an
actuated trial there must be exactly one process authorized to publish the
physical velocity setpoint. That arbiter must provide all of the following:

- explicit, mutually exclusive modes for manual, mission and UUV follow;
- manual and hardware-kill priority over every autonomous source;
- per-source heartbeat and source-timestamp freshness checks;
- zero output on startup, mode changes, stale input, invalid input, process
  restart, clock discontinuity and loss of vehicle-state feedback;
- an operator enable separate from ROS node startup and follow configuration;
- no automatic re-enable after a timeout, rejected fix or killswitch event;
- bounded acceleration as well as bounded speed for the first powered trial;
- status reporting of selected source, gate state, rejection reason, input age
  and final limited command;
- a launch-time or CI assertion that no other node publishes to the physical
  setpoint topic.

The preview topic is an observation input, not permission to actuate. A future
arbiter must consume a separately reviewed command interface; remapping
`/usv/follow/cmd_vel_preview` to MAVROS is explicitly forbidden.

## Acceptance checklist

### A. Offline and ROS graph, no hardware

- [ ] Unit, lint and integration tests pass from a clean build.
- [ ] Fake dropout produces `lost_target` and an exactly zero preview command.
- [ ] Recovery requires the configured number of new consecutive fixes.
- [ ] Wrong frame, stale/future stamp, invalid covariance and impossible jump
      each produce zero.
- [ ] Recorded CSV/bag replay reproduces decisions and rejection reasons.
- [ ] Graph inspection shows no preview publisher on any MAVROS topic.

### B. X150 bench/pool capture, propellers removed

- [ ] Firmware, remote model, IDs and all used CIDs are recorded.
- [ ] Decoder fixtures cover valid, truncated, corrupt and unknown frames.
- [ ] Forward/left/up signs and the X150 mounting transform are independently
      measured and reviewed by a second operator.
- [ ] Timestamps, QoS compatibility, rate, gaps, latency and bag loss meet the
      declared limits.
- [ ] Stationary dispersion, covariance and all provisional quality thresholds
      are justified from the retained dataset.
- [ ] Loss and reacquisition remain fail-safe with the real driver.

### C. Actuated trial

- [ ] A separate review has approved the arbiter, gate and physical killswitch.
- [ ] Only the arbiter publishes the physical setpoint.
- [ ] A dry test proves startup, timeout, source switch and process death all
      produce neutral output.
- [ ] The first test uses a restrained vehicle, a clear abort operator and
      independently limited speed, acceleration and test area.
- [ ] Actuation is enabled only for this test; it is not a default launch
      parameter and is not restored automatically after an abort.

Failure of any applicable item keeps the system in preview mode.

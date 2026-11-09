# USBL relative-position test interface

The real SeaTrac decoder and the hardware-free simulator should eventually
feed the same ROS 2 interface:

- Topic: `/usbl/relative_pose`
- Type: `geometry_msgs/msg/PoseWithCovarianceStamped`
- Position units: metres
- Time: acquisition time, not reception time when the device provides it
- Covariance: position covariance in the 6x6 ROS pose-covariance matrix

For the current simulator and CSV replay, `header.frame_id` is `base_link`,
with `x` forward, `y` left and `z` up on the USV. The lever arm from the
USV-mounted X150 to `base_link`, physical mounting axes and SeaTrac ENU/NED
conversion must be verified before real decoded data uses this topic.

The intended hardware arrangement is an X150 on the USV tracking an X110 or
X010 on the UUV. The `body_forward_m`/`body_left_m`/`body_up_m` columns used by
the replay are already normalized into `base_link`. They must
not be interpreted as the native Easting/Northing/Depth fields emitted by
SeaTrac. The loader intentionally rejects files containing only native
coordinates: heading, sensor mounting and signs must be measured first.
Both fake and replay data must therefore arrive at the controller already in
`base_link`; the controller does not rotate, guess or repair the measurement
frame. The validator checks numeric and quality constraints, not whether an
upstream coordinate transformation was physically correct.

No consumer may keep moving from an old sample. The initial follower treats a
fix older than 2.5 seconds as lost and publishes a zero preview command.

`fake_usbl_node` is a contract simulator only. It does not emulate the SeaTrac
wire protocol. The real range/bearing decoder remains pending until frames from
the installed X150/X110 firmware have been captured and identified.

## CSV replay

The replay passes every row through the hardware-independent validator before
publishing. The required columns are `timestamp_s` (or `stamp_s`),
`body_forward_m` and `body_left_m`. `body_up_m` defaults to zero. Recommended
capture/conversion columns are:

```text
timestamp_s,body_forward_m,body_left_m,body_up_m,beacon_id,range_m,fit_error,rssi_db,position_valid,position_filter_error
```

Run the installed node with:

```bash
ros2 run seatrac_ros2 seatrac_csv_replay --ros-args \
  -p csv_path:=/absolute/path/fixes.csv \
  -p expected_beacon_id:=2
```

The sample file is installed at
`share/seatrac_ros2/config/example_usbl_fixes.csv`. Covariance is deliberately
non-zero and conservative: horizontal uncertainty grows with range and fit
error, vertical uncertainty starts at 0.25 m, and orientation is marked as
unknown. The replay never claims to recover UUV orientation from USBL.

## Offline capture analysis

Analyze the normalized CSV before replaying it or using it to tune safety
thresholds:

```bash
ros2 run seatrac_ros2 seatrac_csv_analyze \
  /absolute/path/fixes.csv \
  --expected-beacon-id 2 \
  --stationary
```

The command returns exit status `0` for `GO`, `2` for `NO-GO`, and `1` when
the input or configuration is invalid. It reports accepted, degraded and
rejected fixes; rejection reasons; effective update rate; median, 95th
percentile and maximum intervals; and gaps at or above the configured stale
timeout. With `--stationary`, it also reports the 95th-percentile horizontal
distance from the median position. Use `--json` for machine-readable output.

Defaults match the provisional preview criteria: at least three accepted
fixes spanning at least 60 seconds, less than five percent rejected samples,
no interval at or above 2.5 s, and at most 0.10 m of 95th-percentile horizontal
error for a stationary test. `--minimum-duration-s` can change the duration
for non-acceptance exploratory captures; do not lower it for a final pool
verdict. These thresholds are provisional until real X150/X110 or X010
recordings are available.

This analysis cannot prove that mounting, axes or signs are correct. A CSV
must contain positions already transformed into `base_link`, and known
physical movements must still be checked before any actuator is enabled.

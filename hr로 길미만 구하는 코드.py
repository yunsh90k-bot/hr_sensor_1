import serial
import math
import re
import matplotlib.pyplot as plt

PORT = "COM4"
BAUD = 9600

EXPECTED_LENGTH = {
    "D0": 17,
}

MIN_DIST = 10
MAX_DIST = 15
TURN_JUMP_DEG = 15
MIN_POINTS = 4
MAX_VALID_THETA = 120

ser = serial.Serial(PORT, BAUD, timeout=1)
names = ["D0"]


def make_sensor():
    return {
        "detecting": False,
        "start_angle": 0,
        "end_angle": 0,
        "count": 0,
        "dist_list": [],
        "points": [],
    }


sensors = {name: make_sensor() for name in names}
all_points = {name: [] for name in names}
results = {"D0": 0}
current_dist = {"D0": 0}
current_angle = 0
prev_angle = 0


def parse_line(line):
    values = dict(re.findall(r"(A|D0)=(\d+)", line))

    if not all(k in values for k in ["A", "D0"]):
        return None

    return {
        "A": int(values["A"]),
        "D0": int(values["D0"]),
    }


def reset_sensor(data):
    data["detecting"] = False
    data["start_angle"] = 0
    data["end_angle"] = 0
    data["count"] = 0
    data["dist_list"] = []
    data["points"] = []


def polar_to_xy(angle_deg, dist):
    theta = math.radians(angle_deg)
    x = dist * math.cos(theta)
    y = dist * math.sin(theta)
    return x, y


def remove_outliers(points):
    if len(points) < 5:
        return points

    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)

    ds = [math.hypot(x - cx, y - cy) for x, y in points]
    sorted_ds = sorted(ds)
    limit = sorted_ds[int(len(sorted_ds) * 0.8)]

    return [
        p for p, d in zip(points, ds)
        if d <= limit
    ]


def calc_length_from_points(points):
    points = remove_outliers(points)

    if len(points) < 2:
        return 0

    max_len = 0

    for i in range(len(points)):
        x1, y1 = points[i]

        for j in range(i + 1, len(points)):
            x2, y2 = points[j]
            d = math.hypot(x2 - x1, y2 - y1)

            if d > max_len:
                max_len = d

    return max_len


def save_result(name, data):
    if data["count"] < MIN_POINTS:
        reset_sensor(data)
        return

    theta_deg = abs(data["end_angle"] - data["start_angle"])

    if theta_deg > MAX_VALID_THETA:
        print(f"\n[{name}] skip: theta={theta_deg}deg")
        reset_sensor(data)
        return

    length = calc_length_from_points(data["points"])

    if length <= 0:
        reset_sensor(data)
        return

    avg_dist = sum(data["dist_list"]) / len(data["dist_list"])

    print(
        f"\n[{name}] count={data['count']} "
        f"theta={theta_deg}deg "
        f"avg_dist={avg_dist:.1f}cm "
        f"L={length:.2f}cm"
    )

    results[name] = length
    reset_sensor(data)


plt.ion()
fig, (ax_len, ax_shape) = plt.subplots(2, 1, figsize=(8, 8))

while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            plt.pause(0.001)
            continue

        parsed = parse_line(line)

        if parsed is None:
            plt.pause(0.001)
            continue

        prev_angle = current_angle
        current_angle = parsed["A"]
        angle_diff = abs(current_angle - prev_angle)

        is_turning = angle_diff > TURN_JUMP_DEG

        for name in names:
            dist = parsed[name]
            current_dist[name] = dist
            data = sensors[name]

            print(f"RAW A={current_angle} {name}={dist}")

            detected = MIN_DIST <= dist <= MAX_DIST

            if is_turning:
                if data["detecting"]:
                    reset_sensor(data)
                continue

            if detected:
                point = polar_to_xy(current_angle, dist)

                all_points[name].append(point)

                if len(all_points[name]) > 1000:
                    all_points[name] = all_points[name][-1000:]

                if not data["detecting"]:
                    data["detecting"] = True
                    data["start_angle"] = current_angle
                    data["end_angle"] = current_angle
                    data["count"] = 1
                    data["dist_list"] = [dist]
                    data["points"] = [point]
                else:
                    data["end_angle"] = current_angle
                    data["count"] += 1
                    data["dist_list"].append(dist)
                    data["points"].append(point)

            else:
                if data["detecting"]:
                    save_result(name, data)

        ax_len.clear()

        measured_length = [results[name] for name in names]
        expected_length = [EXPECTED_LENGTH[name] for name in names]

        ax_len.bar(names, measured_length, label="measured length")
        ax_len.plot(
            names,
            expected_length,
            marker="o",
            linestyle="--",
            label="expected length"
        )

        ax_len.set_ylim(0, 30)
        ax_len.set_ylabel("length cm")
        ax_len.set_title(f"Length View | Angle = {current_angle} deg")
        ax_len.grid(True)
        ax_len.legend()

        for i, name in enumerate(names):
            ax_len.text(
                i,
                measured_length[i] + 0.5,
                f"{measured_length[i]:.2f}cm",
                ha="center"
            )

            if sensors[name]["detecting"]:
                ax_len.text(i, 27, "measuring", ha="center")

            ax_len.text(
                i,
                -2,
                f"dist={current_dist[name]}cm",
                ha="center",
                fontsize=9
            )

        ax_shape.clear()
        ax_shape.set_title("Realtime Detected Shape D0 only")
        ax_shape.set_xlabel("x cm")
        ax_shape.set_ylabel("y cm")
        ax_shape.grid(True)
        ax_shape.set_xlim(-35, 35)
        ax_shape.set_ylim(-5, 35)
        ax_shape.set_aspect("equal", adjustable="box")
        ax_shape.axhline(0)
        ax_shape.axvline(0)

        for name in names:
            pts = all_points[name]

            if pts:
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                ax_shape.scatter(xs, ys, s=8, label=name)

        ax_shape.legend()
        plt.pause(0.001)

    except KeyboardInterrupt:
        print("\n프로그램 종료")
        ser.close()
        break

    except Exception as e:
        print("error:", e)

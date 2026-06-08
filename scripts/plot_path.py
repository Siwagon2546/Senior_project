import json
import argparse
import os
import math
import matplotlib.pyplot as plt


def load_poses_from_json(file_path):
    """
    Load poses from JSON file.
    Expected format:
    {
        "poses": [
            {"x": 1.0, "y": 2.0, "yaw": 90.0}
        ]
    }
    """
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if "poses" not in data:
        raise ValueError(f"File {file_path} does not contain 'poses' key")

    poses = data["poses"]

    x_list = []
    y_list = []
    yaw_list = []

    for i, pose in enumerate(poses):
        if "x" not in pose or "y" not in pose:
            raise ValueError(f"Pose index {i} in {file_path} missing x or y")

        x_list.append(float(pose["x"]))
        y_list.append(float(pose["y"]))
        yaw_list.append(float(pose.get("yaw", 0.0)))

    return x_list, y_list, yaw_list


def plot_yaw_arrows(x_list, y_list, yaw_list, arrow_length=0.8):
    """
    Plot small arrows showing robot yaw direction.
    yaw is assumed to be in degrees.
    """
    for x, y, yaw_deg in zip(x_list, y_list, yaw_list):
        yaw_rad = math.radians(yaw_deg)

        dx = arrow_length * math.cos(yaw_rad)
        dy = arrow_length * math.sin(yaw_rad)

        plt.arrow(
            x,
            y,
            dx,
            dy,
            head_width=0.25,
            head_length=0.35,
            length_includes_head=True,
            alpha=0.7
        )


def plot_json_paths(file_paths, show_yaw=False, save_path=None):
    plt.figure(figsize=(10, 8))

    for file_path in file_paths:
        x_list, y_list, yaw_list = load_poses_from_json(file_path)

        file_name = os.path.basename(file_path)

        # Plot line and points
        plt.plot(
            x_list,
            y_list,
            marker="o",
            linewidth=2,
            markersize=5,
            label=file_name
        )

        # Mark start point
        plt.scatter(
            x_list[0],
            y_list[0],
            marker="s",
            s=100,
            label=f"{file_name} Start"
        )

        # Mark end point
        plt.scatter(
            x_list[-1],
            y_list[-1],
            marker="X",
            s=120,
            label=f"{file_name} End"
        )

        # Optional yaw arrows
        if show_yaw:
            plot_yaw_arrows(x_list, y_list, yaw_list)

    plt.title("XY Path Plot from JSON Files")
    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"Saved plot to: {save_path}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Plot XY paths from one or more JSON files."
    )

    parser.add_argument(
        "files",
        nargs="+",
        help="JSON file paths to plot"
    )

    parser.add_argument(
        "--yaw",
        action="store_true",
        help="Show yaw direction arrows"
    )

    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="Save plot image path, example: output.png"
    )

    args = parser.parse_args()

    plot_json_paths(
        file_paths=args.files,
        show_yaw=args.yaw,
        save_path=args.save
    )


if __name__ == "__main__":
    main()
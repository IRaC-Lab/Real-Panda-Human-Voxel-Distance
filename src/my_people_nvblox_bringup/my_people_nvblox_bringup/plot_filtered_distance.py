"""Plot raw and filtered Panda-to-human distances with matplotlib."""

from collections import deque
from datetime import datetime
from pathlib import Path
import time

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32


class FilteredDistancePlotter(Node):
    def __init__(self) -> None:
        super().__init__('filtered_distance_plotter')
        self.started_at = time.monotonic()
        self.raw_times = deque(maxlen=600)
        self.raw_distances = deque(maxlen=600)
        self.filtered_times = deque(maxlen=600)
        self.filtered_distances = deque(maxlen=600)
        self.create_subscription(
            Float32, '/closest_panda_human/raw_distance', self.raw_callback, 10)
        self.create_subscription(
            Float32, '/closest_panda_human/distance',
            self.filtered_callback, 10)

    def raw_callback(self, message: Float32) -> None:
        self.raw_times.append(time.monotonic() - self.started_at)
        self.raw_distances.append(float(message.data))

    def filtered_callback(self, message: Float32) -> None:
        self.filtered_times.append(time.monotonic() - self.started_at)
        self.filtered_distances.append(float(message.data))


def run_plot(show_raw: bool, args=None) -> None:
    rclpy.init(args=args)
    node = FilteredDistancePlotter()
    figure, axes = plt.subplots()
    raw_line = None
    if show_raw:
        raw_line, = axes.plot([], [], color='tab:red', label='d_raw')
    filtered_line, = axes.plot(
        [], [], color='tab:blue', label='d_filtered')
    axes.set(title='Panda–Human Distance',
             xlabel='Time [s]', ylabel='Distance [m]')
    axes.grid(True)
    axes.legend()
    plt.show(block=False)

    try:
        while rclpy.ok() and plt.fignum_exists(figure.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            if raw_line is not None:
                raw_line.set_data(node.raw_times, node.raw_distances)
            filtered_line.set_data(
                node.filtered_times, node.filtered_distances)
            axes.relim()
            axes.autoscale_view()
            plt.pause(0.01)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        plot_name = 'distance_comparison' if show_raw else 'filtered_distance'
        output_directory = Path.home() / 'my_ws' / 'graph'
        output_directory.mkdir(parents=True, exist_ok=True)
        output_path = output_directory / (
            f'{plot_name}_{datetime.now():%Y%m%d_%H%M%S}.png')
        figure.savefig(output_path, dpi=200, bbox_inches='tight')
        print(f'Saved graph: {output_path}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    run_plot(False, args)


def main_both(args=None) -> None:
    run_plot(True, args)


if __name__ == '__main__':
    main()

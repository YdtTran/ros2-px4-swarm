"""
Chase camera: uses Gazebo's built-in /gui/follow service to lock onto a drone.

Retries every RETRY_SEC until Gazebo accepts the request — no fixed sleep.
Drone name and follow offset are ROS 2 parameters.
"""
import subprocess

import rclpy
from rclpy.node import Node


RETRY_SEC = 2.0


class GzChaseCam(Node):

    def __init__(self):
        super().__init__('gz_chase_cam')
        self.declare_parameter('drone_name', 'x500_0')
        self.declare_parameter('offset_x', -4.0)
        self.declare_parameter('offset_y',  0.0)
        self.declare_parameter('offset_z',  1.5)

        self._drone = self.get_parameter('drone_name').value
        self._ox    = self.get_parameter('offset_x').value
        self._oy    = self.get_parameter('offset_y').value
        self._oz    = self.get_parameter('offset_z').value

        self._attempt = 0
        self._timer = self.create_timer(RETRY_SEC, self._try_follow)
        self.get_logger().info(f'Waiting for Gazebo — will follow "{self._drone}"')

    def _gz(self, service: str, reqtype: str, req: str):
        """Return (ok, stdout, stderr)."""
        result = subprocess.run(
            ['gz', 'service', '-s', service,
             '--reqtype', reqtype,
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', req],
            capture_output=True,
            text=True,
        )
        ok = result.returncode == 0 and (
            'data: true' in result.stdout or 'data:true' in result.stdout
        )
        return ok, result.stdout.strip(), result.stderr.strip()

    def _try_follow(self):
        self._attempt += 1
        ok, out, err = self._gz(
            '/gui/follow',
            'gz.msgs.StringMsg',
            f'data: "{self._drone}"',
        )
        if not ok:
            # Log detail on first attempt and every 5th after to avoid spam
            if self._attempt == 1 or self._attempt % 5 == 0:
                self.get_logger().info(
                    f'[attempt {self._attempt}] /gui/follow not ready — '
                    f'out={out!r} err={err!r}'
                )
            return

        self._gz(
            '/gui/follow/offset',
            'gz.msgs.Vector3d',
            f'x: {self._ox}, y: {self._oy}, z: {self._oz}',
        )
        self.get_logger().info(
            f'Chase cam set — following "{self._drone}", '
            f'offset ({self._ox}, {self._oy}, {self._oz})'
        )
        self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GzChaseCam())


if __name__ == '__main__':
    main()

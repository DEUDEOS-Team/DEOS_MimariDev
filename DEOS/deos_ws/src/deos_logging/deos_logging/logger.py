import logging
from pathlib import Path
from datetime import datetime

from deos_logging.cleanup import cleanup_logs


class DeosLogger:

    def __init__(self, ros_logger, node_name: str):
        self.ros_logger = ros_logger
        self.node_name = node_name

        cleanup_logs(hours=12)

        self.log_dir = Path("logs") / node_name
        self.log_dir.mkdir(parents=True, exist_ok=True)

        now = datetime.now()
        millisecond = f"{now.microsecond // 1000:03d}"
        filename = f"{now.strftime('%Y-%m-%d_%H-%M-%S')}-{millisecond}.log"
        self.log_file = self.log_dir / filename

        self.logger = logging.getLogger(node_name)
        self.logger.setLevel(logging.DEBUG)

        if not self.logger.handlers:
            file_handler = logging.FileHandler(self.log_file, encoding="utf-8")
            file_handler.setFormatter(logging.Formatter(
                fmt="%(asctime)s.%(msecs)03d | %(levelname)s | %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            ))
            self.logger.addHandler(file_handler)

        self.logger.propagate = False

    def debug(self, message: str):
        self.logger.debug(message)
        self.ros_logger.debug(message)

    def info(self, message: str):
        self.logger.info(message)
        self.ros_logger.info(message)

    def warning(self, message: str):
        self.logger.warning(message)
        self.ros_logger.warning(message)

    def error(self, message: str):
        self.logger.error(message)
        self.ros_logger.error(message)

    def critical(self, message: str):
        self.logger.critical(message)
        self.ros_logger.fatal(message)

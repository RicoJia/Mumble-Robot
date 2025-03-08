import logging


# ANSI Color Codes for Console Output
COLOR_RESET = "\033[0m"
COLOR_INFO = "\033[92m"  # Green
COLOR_WARNING = "\033[93m"  # Yellow
COLOR_ERROR = "\033[91m"  # Red
COLOR_DEBUG = "\033[94m"  # Blue


class ColoredFormatter(logging.Formatter):
    COLORS = {
        logging.INFO: COLOR_INFO,
        logging.WARNING: COLOR_WARNING,
        logging.ERROR: COLOR_ERROR,
        logging.DEBUG: COLOR_DEBUG,
    }

    def format(self, record):
        log_color = self.COLORS.get(record.levelno, COLOR_RESET)
        reset_color = COLOR_RESET

        # Get the formatted message from the base class
        message = super().format(record)

        # Color the log level (only the first occurrence)
        colored_level = f"{log_color}[{record.levelname}]{reset_color}"
        message = message.replace(f"[{record.levelname}]", colored_level, 1)

        # Assume the timestamp is the first token in the formatted message
        parts = message.split(" ", 1)
        if len(parts) == 2:
            timestamp, rest = parts
            colored_timestamp = f"{log_color}{timestamp}{reset_color}"
            message = f"{colored_timestamp} {rest}"

        return message


# Logger setup with color support
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)

def get_logger(logger_name:str=""):
    logger = logging.getLogger(logger_name)
    handler = logging.StreamHandler()
    handler.setFormatter(ColoredFormatter("%(asctime)s [%(levelname)s] %(message)s"))
    logger.handlers = [handler]
    logger.propagate = False
    return logger
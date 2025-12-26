import logging
import sys
from datetime import datetime
from typing import Any, Dict
import json


class CustomFormatter(logging.Formatter):
    """Custom formatter to add additional context to log messages."""

    def format(self, record: logging.LogRecord) -> str:
        # Add timestamp and log level to the record
        log_record = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }

        # Add any extra fields that were passed to the logger
        for key, value in record.__dict__.items():
            if key not in log_record and key not in ["name", "msg", "args", "levelname", "levelno", "pathname", "filename", "module", "lineno", "funcName", "created", "msecs", "relativeCreated", "thread", "threadName", "processName", "process", "getMessage", "exc_info", "exc_text", "stack_info"]:
                log_record[key] = value

        # Add exception info if present
        if record.exc_info:
            log_record["exception"] = self.formatException(record.exc_info)

        return json.dumps(log_record)


def setup_logging():
    """Set up logging configuration for the application."""
    # Create a custom logger
    logger = logging.getLogger("ai_book")
    logger.setLevel(logging.INFO)

    # Create handlers
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)

    # Create formatters and add it to handlers
    formatter = CustomFormatter()
    console_handler.setFormatter(formatter)

    # Add handlers to the logger
    logger.addHandler(console_handler)

    # Prevent duplicate logs if the logger is used in multiple places
    logger.propagate = False

    return logger


# Global logger instance
logger = setup_logging()


def log_api_call(endpoint: str, method: str, user_id: str = None, **kwargs):
    """Log API calls with relevant context."""
    logger.info(
        f"API call: {method} {endpoint}",
        extra={
            "endpoint": endpoint,
            "method": method,
            "user_id": user_id,
            **kwargs
        }
    )


def log_error(error: Exception, context: str = "", **kwargs):
    """Log errors with context."""
    logger.error(
        f"Error in {context}: {str(error)}",
        extra={
            "context": context,
            "error_type": type(error).__name__,
            **kwargs
        },
        exc_info=True  # This will include the traceback
    )
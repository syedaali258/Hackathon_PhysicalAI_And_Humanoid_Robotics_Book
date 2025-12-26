from fastapi import Request, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import time
from typing import Dict, Optional
import hashlib
import hmac
import os
from api.utils.logging import logger


class SecurityHeaders:
    """Utility class for adding security headers to responses."""

    @staticmethod
    def add_security_headers(request: Request, response):
        """Add security headers to the response."""
        # Content Security Policy
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; "
            "script-src 'self' 'unsafe-inline' 'unsafe-eval'; "
            "style-src 'self' 'unsafe-inline'; "
            "img-src 'self' data: https:; "
            "font-src 'self' data:; "
            "connect-src 'self' https://api.openai.com; "
            "frame-ancestors 'none';"
        )

        # X-Frame-Options
        response.headers["X-Frame-Options"] = "DENY"

        # X-Content-Type-Options
        response.headers["X-Content-Type-Options"] = "nosniff"

        # X-XSS-Protection
        response.headers["X-XSS-Protection"] = "1; mode=block"

        # Referrer-Policy
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"

        # Strict-Transport-Security
        response.headers["Strict-Transport-Security"] = (
            "max-age=31536000; includeSubDomains"
        )

        return response


class RateLimiter:
    """Simple rate limiter to prevent abuse."""

    def __init__(self, requests: int = 100, window: int = 3600):
        self.requests = requests
        self.window = window  # in seconds
        self.requests_log: Dict[str, list] = {}

    def is_allowed(self, identifier: str) -> bool:
        """Check if the identifier is allowed to make a request."""
        current_time = time.time()

        if identifier not in self.requests_log:
            self.requests_log[identifier] = []

        # Remove requests that are outside the current window
        self.requests_log[identifier] = [
            req_time for req_time in self.requests_log[identifier]
            if current_time - req_time < self.window
        ]

        # Check if the limit is exceeded
        if len(self.requests_log[identifier]) >= self.requests:
            return False

        # Add current request to log
        self.requests_log[identifier].append(current_time)
        return True


class APIKeyValidator:
    """Utility class for validating API keys."""

    def __init__(self):
        self.valid_keys = {
            os.getenv("API_KEY", "test-key-for-development"): "default_user"
        }

    def validate_api_key(self, api_key: str) -> Optional[str]:
        """Validate the API key and return user ID if valid."""
        if not api_key:
            return None

        if api_key in self.valid_keys:
            return self.valid_keys[api_key]

        return None


# Global instances
rate_limiter = RateLimiter(requests=100, window=3600)  # 100 requests per hour
api_key_validator = APIKeyValidator()
security_scheme = HTTPBearer(auto_error=False)


def validate_request_content(request: Request, content: str) -> bool:
    """Validate request content for potentially harmful patterns."""
    if not content:
        return True

    # Check for potential injection patterns
    harmful_patterns = [
        r"(\bselect\b|\binsert\b|\bupdate\b|\bdelete\b|\bdrop\b|\bunion\b|\bexec\b)",
        r"(script|javascript|vbscript|onload|onerror)",
        r"(\.\.\/|\.\.\\)",  # Directory traversal
    ]

    content_lower = content.lower()
    for pattern in harmful_patterns:
        if __import__('re').search(pattern, content_lower):
            logger.warning(
                f"Potentially harmful content detected",
                extra={
                    "client_ip": request.client.host if request.client else "unknown",
                    "pattern": pattern,
                    "content_preview": content[:100]
                }
            )
            return False

    return True
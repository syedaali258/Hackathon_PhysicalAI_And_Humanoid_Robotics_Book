import pytest
import sys
import os

# Add the api directory to the Python path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


@pytest.fixture(autouse=True)
def setup_test_environment():
    """Set up the test environment."""
    # Set environment variables for testing
    os.environ.setdefault("OPENAI_API_KEY", "test-key-for-testing")
    os.environ.setdefault("API_DEBUG", "true")

    # Yield control back to the test
    yield

    # Teardown (if needed)
    # Clean up any test-specific environment variables
    if "OPENAI_API_KEY" in os.environ:
        del os.environ["OPENAI_API_KEY"]
    if "API_DEBUG" in os.environ:
        del os.environ["API_DEBUG"]
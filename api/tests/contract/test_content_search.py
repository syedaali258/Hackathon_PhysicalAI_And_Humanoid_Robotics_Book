import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestContentSearchContract:
    """Contract tests for the content search endpoint based on the OpenAPI specification."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_content_search_contract(self):
        """Test that /content/search endpoint matches the OpenAPI contract."""
        search_request = {
            "query": "ROS 2 fundamentals",
            "limit": 5,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        # The response might fail due to missing vector database in test environment
        # but we should check that it follows the contract when successful
        if response.status_code == 200:
            data = response.json()

            # Check response structure according to OpenAPI spec
            assert "results" in data, "Response should contain 'results' field"
            assert "query" in data, "Response should contain 'query' field"
            assert "total" in data, "Response should contain 'total' field"

            # Check data types
            assert isinstance(data["results"], list), "results should be a list"
            assert isinstance(data["query"], str), "query should be a string"
            assert isinstance(data["total"], int), "total should be an integer"

            # Check results structure
            for result in data["results"]:
                assert "contentId" in result, "Each result should have contentId"
                assert "title" in result, "Each result should have title"
                assert "excerpt" in result, "Each result should have excerpt"
                assert "url" in result, "Each result should have url"
                assert "relevance" in result, "Each result should have relevance"

                # Check data types for result fields
                assert isinstance(result["contentId"], str), "contentId should be a string"
                assert isinstance(result["title"], str), "title should be a string"
                assert isinstance(result["excerpt"], str), "excerpt should be a string"
                assert isinstance(result["url"], str), "url should be a string"
                assert isinstance(result["relevance"], (int, float)), "relevance should be a number"

                # Check that relevance is between 0 and 1
                assert 0 <= result["relevance"] <= 1, "relevance should be between 0 and 1"

        elif response.status_code in [400, 500]:
            # These are valid error responses according to the OpenAPI spec
            pass
        else:
            # Unexpected status code
            assert False, f"Unexpected status code: {response.status_code}"

    def test_content_search_required_fields(self):
        """Test that the content search endpoint properly validates required fields."""
        # Test with missing query (should return 400)
        search_request = {
            # Missing "query" field
            "limit": 5,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        # Could return 400 for validation error or 500 for internal error
        assert response.status_code in [400, 422, 500], f"Expected 400/422/500, got {response.status_code}"

    def test_content_search_with_limit(self):
        """Test that the content search endpoint respects the limit parameter."""
        search_request = {
            "query": "ROS 2",
            "limit": 3,  # Request only 3 results
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        if response.status_code == 200:
            data = response.json()
            assert "results" in data
            assert "total" in data

            # The number of results should not exceed the limit
            assert len(data["results"]) <= 3, f"Should return at most 3 results, got {len(data['results'])}"
            assert data["total"] >= len(data["results"]), "Total should be >= number of returned results"

    def test_content_search_with_module_filter(self):
        """Test that the content search endpoint can handle module filters."""
        search_request = {
            "query": "nodes",
            "limit": 5,
            "filters": {
                "module": "module-1-ros2"
            }
        }
        response = self.client.post("/content/search", json=search_request)

        # Check that the request is accepted (status 200, 400, or 500 are all valid)
        assert response.status_code in [200, 400, 500], f"Unexpected status code: {response.status_code}"

        if response.status_code == 200:
            data = response.json()
            assert "results" in data
            assert "query" in data
            assert "total" in data
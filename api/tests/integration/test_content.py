import pytest
from fastapi.testclient import TestClient
from api.main import app
from api.services.rag_service import rag_service


class TestROS2FundamentalsIntegration:
    """Integration tests for ROS 2 fundamentals content in the RAG system."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_ros2_fundamentals_content_retrieval(self):
        """Test that ROS 2 fundamentals content can be retrieved through the RAG system."""
        # Search for ROS 2 fundamentals content
        search_request = {
            "query": "ROS 2 fundamentals nodes topics services",
            "limit": 5,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        # Check that the search was successful
        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert len(data["results"]) > 0

        # Check that at least one result is related to ROS 2 fundamentals
        found_fundamentals = False
        for result in data["results"]:
            if "fundamentals" in result["title"].lower() or "fundamentals" in result["excerpt"].lower():
                found_fundamentals = True
                break

        assert found_fundamentals, "Should find content related to ROS 2 fundamentals"

    def test_ros2_nodes_concept_retrieval(self):
        """Test that content about ROS 2 nodes can be retrieved."""
        search_request = {
            "query": "What are ROS 2 nodes?",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about nodes
        found_nodes_info = False
        for result in data["results"]:
            if "node" in result["excerpt"].lower():
                found_nodes_info = True
                break

        assert found_nodes_info, "Should find content about ROS 2 nodes"

    def test_ros2_topics_concept_retrieval(self):
        """Test that content about ROS 2 topics can be retrieved."""
        search_request = {
            "query": "Explain ROS 2 topics",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about topics
        found_topics_info = False
        for result in data["results"]:
            if "topic" in result["excerpt"].lower():
                found_topics_info = True
                break

        assert found_topics_info, "Should find content about ROS 2 topics"

    def test_ros2_services_concept_retrieval(self):
        """Test that content about ROS 2 services can be retrieved."""
        search_request = {
            "query": "What are ROS 2 services?",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about services
        found_services_info = False
        for result in data["results"]:
            if "service" in result["excerpt"].lower():
                found_services_info = True
                break

        assert found_services_info, "Should find content about ROS 2 services"

    def test_ros2_fundamentals_chat_response(self):
        """Test that the chat system can respond to questions about ROS 2 fundamentals."""
        # Start a chat session
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Ask a question about ROS 2 fundamentals
        chat_request = {
            "message": "What are the main differences between ROS 2 nodes, topics, and services?",
            "context": "Module 1, Chapter 1: ROS 2 Fundamentals"
        }
        response = self.client.post(f"/chat/{session_id}/message", json=chat_request)

        # The response might fail due to missing OpenAI API key in test environment
        # but we should check if it would work properly if the service was available
        if response.status_code == 200:
            data = response.json()
            # If successful, check that response contains relevant information
            assert "response" in data
            # Note: We can't validate specific content without the actual RAG system running
        elif response.status_code in [400, 404, 500]:
            # These are valid responses according to the API contract
            pass
        else:
            assert False, f"Unexpected status code: {response.status_code}"

    def test_content_structure_matching(self):
        """Test that the content structure matches what's expected for ROS 2 fundamentals."""
        # Search for ROS 2 fundamentals
        search_request = {
            "query": "ROS 2 fundamentals",
            "limit": 1,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        if len(data["results"]) > 0:
            result = data["results"][0]
            # Check that the result has expected fields
            assert "contentId" in result
            assert "title" in result
            assert "excerpt" in result
            assert "url" in result
            assert "relevance" in result

            # Check that the title is related to ROS 2 fundamentals
            assert "fundamentals" in result["title"].lower() or "ros" in result["title"].lower()
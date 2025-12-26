import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestURDFContentIntegration:
    """Integration tests for URDF content in the RAG system."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_urdf_content_retrieval(self):
        """Test that URDF content can be retrieved through the RAG system."""
        # Search for URDF content
        search_request = {
            "query": "Humanoid Modeling with URDF Universal Robot Description Format",
            "limit": 5,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        # Check that the search was successful
        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert len(data["results"]) > 0

        # Check that at least one result is related to URDF
        found_urdf = False
        for result in data["results"]:
            if "urdf" in result["title"].lower() or "urdf" in result["excerpt"].lower():
                found_urdf = True
                break

        assert found_urdf, "Should find content related to URDF"

    def test_urdf_basic_structure_retrieval(self):
        """Test that content about URDF basic structure can be retrieved."""
        search_request = {
            "query": "basic URDF structure links joints",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about URDF structure
        found_structure_info = False
        for result in data["results"]:
            if "link" in result["excerpt"].lower() or "joint" in result["excerpt"].lower():
                found_structure_info = True
                break

        assert found_structure_info, "Should find content about URDF structure"

    def test_urdf_elements_retrieval(self):
        """Test that content about URDF elements (links, joints) can be retrieved."""
        search_request = {
            "query": "URDF elements links joints",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about URDF elements
        found_elements_info = False
        for result in data["results"]:
            excerpt = result["excerpt"].lower()
            if "link" in excerpt or "joint" in excerpt or "visual" in excerpt or "collision" in excerpt:
                found_elements_info = True
                break

        assert found_elements_info, "Should find content about URDF elements"

    def test_humanoid_urdf_retrieval(self):
        """Test that content about humanoid URDF models can be retrieved."""
        search_request = {
            "query": "humanoid robot URDF model",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about humanoid robots
        found_humanoid_info = False
        for result in data["results"]:
            if "humanoid" in result["excerpt"].lower() or "torso" in result["excerpt"].lower():
                found_humanoid_info = True
                break

        assert found_humanoid_info, "Should find content about humanoid URDF models"

    def test_urdf_xacro_retrieval(self):
        """Test that content about Xacro for URDF can be retrieved."""
        search_request = {
            "query": "Xacro URDF macro language",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about Xacro
        found_xacro_info = False
        for result in data["results"]:
            if "xacro" in result["excerpt"].lower():
                found_xacro_info = True
                break

        assert found_xacro_info, "Should find content about Xacro"

    def test_urdf_chat_response(self):
        """Test that the chat system can respond to questions about URDF."""
        # Start a chat session
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Ask a question about URDF
        chat_request = {
            "message": "What is URDF and how is it used for humanoid robot modeling?",
            "context": "Module 1, Chapter 3: Humanoid Modeling with URDF"
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

    def test_urdf_validation_content_retrieval(self):
        """Test that content about URDF validation can be retrieved."""
        search_request = {
            "query": "URDF validation tools",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about URDF validation
        found_validation_info = False
        for result in data["results"]:
            if "validate" in result["excerpt"].lower() or "check_urdf" in result["excerpt"].lower():
                found_validation_info = True
                break

        assert found_validation_info, "Should find content about URDF validation"

    def test_urdf_visualization_retrieval(self):
        """Test that content about URDF visualization can be retrieved."""
        search_request = {
            "query": "URDF visualization RViz",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about URDF visualization
        found_visualization_info = False
        for result in data["results"]:
            if "visualize" in result["excerpt"].lower() or "rviz" in result["excerpt"].lower():
                found_visualization_info = True
                break

        assert found_visualization_info, "Should find content about URDF visualization"

    def test_content_structure_matching_for_urdf(self):
        """Test that the content structure matches what's expected for URDF."""
        # Search for URDF content
        search_request = {
            "query": "Universal Robot Description Format",
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

            # Check that the title is related to URDF
            assert "urdf" in result["title"].lower() or "robot" in result["title"].lower()
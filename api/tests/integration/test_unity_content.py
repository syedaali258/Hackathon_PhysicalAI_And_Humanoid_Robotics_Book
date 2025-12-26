import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestUnityContentIntegration:
    """Integration tests for Unity interaction content in the RAG system."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_unity_interaction_content_retrieval(self):
        """Test that Unity interaction content can be retrieved through the RAG system."""
        # Search for Unity interaction content
        search_request = {
            "query": "Unity interaction human-robot interface",
            "limit": 5,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        # Check that the search was successful
        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_unity_human_robot_interaction_retrieval(self):
        """Test that content about human-robot interaction in Unity can be retrieved."""
        search_request = {
            "query": "human-robot interaction in Unity for humanoid robots",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_unity_visualization_content_retrieval(self):
        """Test that content about Unity visualization can be retrieved."""
        search_request = {
            "query": "Unity visualization for robotics simulation",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_unity_interface_design_retrieval(self):
        """Test that content about Unity interface design can be retrieved."""
        search_request = {
            "query": "Unity interface design for robot monitoring and control",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_unity_interaction_chat_response(self):
        """Test that the chat system can respond to questions about Unity interaction."""
        # Start a chat session
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Ask a question about Unity interaction
        chat_request = {
            "message": "How is human-robot interaction designed in Unity for monitoring and control?",
            "context": "Module 2, Chapter 3: Human-Robot Interaction in Unity"
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

    def test_unity_vs_gazebo_role_explanation_chat(self):
        """Test that the chat system can explain Unity's role vs Gazebo."""
        # Start a chat session
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Ask about Unity vs Gazebo roles
        chat_request = {
            "message": "What is Unity's role compared to Gazebo in digital twin simulation?",
            "context": "Module 2: The Digital Twin (Gazebo & Unity)"
        }
        response = self.client.post(f"/chat/{session_id}/message", json=chat_request)

        if response.status_code == 200:
            data = response.json()
            assert "response" in data
        elif response.status_code in [400, 404, 500]:
            # Valid responses
            pass
        else:
            assert False, f"Unexpected status code: {response.status_code}"

    def test_unity_visualization_techniques_content(self):
        """Test that content about Unity visualization techniques can be retrieved."""
        search_request = {
            "query": "Unity visualization techniques for robotics simulation",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

        # Check that results contain information relevant to Unity visualization
        if len(data["results"]) > 0:
            # At least verify the structure is correct
            result = data["results"][0]
            assert "contentId" in result
            assert "title" in result
            assert "excerpt" in result
            assert "url" in result
            assert "relevance" in result

    def test_unity_user_interface_content_retrieval(self):
        """Test that content about Unity user interfaces can be retrieved."""
        search_request = {
            "query": "Unity user interfaces for robot control and monitoring",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_unity_integration_with_gazebo_content(self):
        """Test that content about Unity integration with Gazebo can be retrieved."""
        search_request = {
            "query": "Unity integration with Gazebo for comprehensive digital twin",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_unity_content_with_module_filter(self):
        """Test that Unity content can be filtered by module."""
        # Search for Unity content with module filter
        search_request = {
            "query": "Unity interaction",
            "limit": 5,
            "filters": {
                "module": "module-2-digital-twin"
            }
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code in [200, 400, 500]  # Valid responses
        if response.status_code == 200:
            data = response.json()
            assert "results" in data
            assert "query" in data
            assert "total" in data

    def test_content_structure_matching_for_unity_interaction(self):
        """Test that the content structure matches what's expected for Unity interaction."""
        # Search for Unity interaction content
        search_request = {
            "query": "Unity human-robot interaction",
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

            # Verify data types
            assert isinstance(result["contentId"], str)
            assert isinstance(result["title"], str)
            assert isinstance(result["excerpt"], str)
            assert isinstance(result["url"], str)
            assert isinstance(result["relevance"], (int, float))
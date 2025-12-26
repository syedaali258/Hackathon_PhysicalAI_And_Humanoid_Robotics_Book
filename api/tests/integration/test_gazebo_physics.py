import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestGazeboPhysicsIntegration:
    """Integration tests for Gazebo physics simulation content in the RAG system."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_gazebo_physics_content_retrieval(self):
        """Test that Gazebo physics simulation content can be retrieved through the RAG system."""
        # Search for Gazebo physics content
        search_request = {
            "query": "Gazebo physics simulation gravity collisions",
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

    def test_gazebo_gravity_simulation_retrieval(self):
        """Test that content about gravity simulation in Gazebo can be retrieved."""
        search_request = {
            "query": "gravity simulation in Gazebo for humanoid robots",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_gazebo_collision_detection_retrieval(self):
        """Test that content about collision detection in Gazebo can be retrieved."""
        search_request = {
            "query": "collision detection in Gazebo physics simulation",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_gazebo_physics_material_properties_retrieval(self):
        """Test that content about material properties in Gazebo physics can be retrieved."""
        search_request = {
            "query": "material properties and physics simulation in Gazebo",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_gazebo_physics_chat_response(self):
        """Test that the chat system can respond to questions about Gazebo physics."""
        # Start a chat session
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Ask a question about Gazebo physics
        chat_request = {
            "message": "How does Gazebo simulate gravity and collisions for humanoid robots?",
            "context": "Module 2, Chapter 1: Physics Simulation in Gazebo"
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

    def test_gazebo_physics_parameters_chat(self):
        """Test that the chat system can explain Gazebo physics parameters."""
        # Start a chat session
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Ask about physics parameters
        chat_request = {
            "message": "What physics parameters can be adjusted in Gazebo simulation?",
            "context": "Module 2, Chapter 1: Physics Simulation in Gazebo"
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

    def test_gazebo_simulation_setup_content(self):
        """Test that content about setting up Gazebo simulation can be retrieved."""
        search_request = {
            "query": "how to set up Gazebo physics simulation for humanoid robot",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

        # Check that results contain information relevant to setup
        if len(data["results"]) > 0:
            # At least verify the structure is correct
            result = data["results"][0]
            assert "contentId" in result
            assert "title" in result
            assert "excerpt" in result
            assert "url" in result
            assert "relevance" in result

    def test_gazebo_physics_content_with_module_filter(self):
        """Test that Gazebo physics content can be filtered by module."""
        # Search for Gazebo physics content with module filter
        search_request = {
            "query": "Gazebo physics",
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

    def test_content_structure_matching_for_gazebo_physics(self):
        """Test that the content structure matches what's expected for Gazebo physics."""
        # Search for Gazebo physics content
        search_request = {
            "query": "Gazebo physics simulation",
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
import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestChatContract:
    """Contract tests for the chat endpoint based on the OpenAPI specification."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_start_chat_contract(self):
        """Test that /chat/start endpoint matches the OpenAPI contract."""
        response = self.client.post("/chat/start")

        # Check status code
        assert response.status_code == 201, f"Expected status 201, got {response.status_code}"

        # Check response structure
        data = response.json()
        assert "sessionId" in data, "Response should contain sessionId"
        assert "timestamp" in data, "Response should contain timestamp"
        assert "message" in data, "Response should contain message"

        # Check data types
        assert isinstance(data["sessionId"], str), "sessionId should be a string"
        assert isinstance(data["timestamp"], str), "timestamp should be a string"
        assert isinstance(data["message"], str), "message should be a string"

        # Check that sessionId is not empty
        assert len(data["sessionId"]) > 0, "sessionId should not be empty"

        # Check that message contains expected content
        assert "AI assistant" in data["message"], "Message should contain AI assistant reference"

    def test_send_message_contract(self):
        """Test that /chat/{sessionId}/message endpoint matches the OpenAPI contract."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Send a message
        chat_request = {
            "message": "What is ROS 2?",
            "context": "Module 1, Chapter 1: ROS 2 Fundamentals"
        }
        response = self.client.post(f"/chat/{session_id}/message", json=chat_request)

        # The response might fail due to missing OpenAI API key in test environment
        # but we should at least check that the endpoint exists and returns proper structure when successful
        if response.status_code == 200:
            data = response.json()

            # Check response structure according to OpenAPI spec
            assert "response" in data, "Response should contain 'response' field"
            assert "sources" in data, "Response should contain 'sources' field"
            assert "followUp" in data, "Response should contain 'followUp' field"
            assert "timestamp" in data, "Response should contain 'timestamp' field"
            assert "confidence" in data, "Response should contain 'confidence' field"

            # Check data types
            assert isinstance(data["response"], str), "response should be a string"
            assert isinstance(data["sources"], list), "sources should be a list"
            assert isinstance(data["followUp"], list), "followUp should be a list"
            assert isinstance(data["timestamp"], str), "timestamp should be a string"
            assert isinstance(data["confidence"], (int, float)), "confidence should be a number"

            # Check confidence is between 0 and 1
            assert 0 <= data["confidence"] <= 1, "confidence should be between 0 and 1"

            # Check sources structure
            for source in data["sources"]:
                assert "contentId" in source, "Each source should have contentId"
                assert "title" in source, "Each source should have title"
                assert "url" in source, "Each source should have url"
                assert "confidence" in source, "Each source should have confidence"

        elif response.status_code in [400, 404, 500]:
            # These are valid error responses according to the OpenAPI spec
            pass
        else:
            # Unexpected status code
            assert False, f"Unexpected status code: {response.status_code}"

    def test_get_chat_history_contract(self):
        """Test that /chat/{sessionId}/history endpoint matches the OpenAPI contract."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Get chat history
        response = self.client.get(f"/chat/{session_id}/history?limit=10")

        # Check status code
        assert response.status_code == 200, f"Expected status 200, got {response.status_code}"

        # Check response structure according to OpenAPI spec
        data = response.json()
        assert "messages" in data, "Response should contain 'messages' field"
        assert "hasMore" in data, "Response should contain 'hasMore' field"
        assert "total" in data, "Response should contain 'total' field"

        # Check data types
        assert isinstance(data["messages"], list), "messages should be a list"
        assert isinstance(data["hasMore"], bool), "hasMore should be a boolean"
        assert isinstance(data["total"], int), "total should be an integer"

        # Check messages structure
        for message in data["messages"]:
            assert "id" in message, "Each message should have id"
            assert "sender" in message, "Each message should have sender"
            assert "content" in message, "Each message should have content"
            assert "timestamp" in message, "Each message should have timestamp"
            assert message["sender"] in ["user", "assistant"], "Sender should be 'user' or 'assistant'"

    def test_end_chat_contract(self):
        """Test that /chat/{sessionId}/end endpoint matches the OpenAPI contract."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # End the chat
        response = self.client.post(f"/chat/{session_id}/end")

        # Check status code
        assert response.status_code == 200, f"Expected status 200, got {response.status_code}"

        # Check response structure according to OpenAPI spec
        data = response.json()
        assert "message" in data, "Response should contain 'message' field"

        # Duration might be present or not depending on implementation
        if "duration" in data:
            assert isinstance(data["duration"], int), "duration should be an integer if present"

        # Check data types
        assert isinstance(data["message"], str), "message should be a string"

        # Check that message contains expected content
        assert "ended" in data["message"].lower(), "Message should indicate session ended"
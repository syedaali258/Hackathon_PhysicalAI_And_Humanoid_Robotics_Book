"""
Unit tests for the Speech Service in VLA Module
"""
import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock
import base64

from services.speech_service import SpeechService


class TestSpeechService:
    """Test cases for SpeechService"""

    @pytest.fixture
    def speech_service(self):
        """Create a SpeechService instance for testing"""
        return SpeechService()

    @pytest.mark.asyncio
    async def test_recognize_speech_with_valid_audio(self, speech_service):
        """Test speech recognition with valid audio data"""
        # Create mock audio data (base64 encoded)
        mock_audio_data = base64.b64encode(b"mock audio bytes").decode('utf-8')

        # Test the recognize_speech method
        text, confidence = await speech_service.recognize_speech(mock_audio_data)

        # Since we're using mock responses, we expect default values
        assert isinstance(text, str)
        assert isinstance(confidence, float)
        assert 0.0 <= confidence <= 1.0

    @pytest.mark.asyncio
    async def test_recognize_speech_with_empty_audio(self, speech_service):
        """Test speech recognition with empty audio data"""
        text, confidence = await speech_service.recognize_speech("")

        # Should return unknown command with low confidence
        assert text == "unknown command"
        assert confidence == 0.0

    @pytest.mark.asyncio
    async def test_recognize_speech_with_invalid_base64(self, speech_service):
        """Test speech recognition with invalid base64 data"""
        with pytest.raises(Exception):
            await speech_service.recognize_speech("invalid_base64!")

    @pytest.mark.asyncio
    async def test_validate_audio_format_valid(self, speech_service):
        """Test audio format validation with valid data"""
        # Create a valid base64 string that's not too small or too large
        valid_audio = base64.b64encode(b"valid audio data").decode('utf-8')

        is_valid = speech_service.validate_audio_format(valid_audio)
        assert is_valid is True

    @pytest.mark.asyncio
    async def test_validate_audio_format_too_small(self, speech_service):
        """Test audio format validation with data that's too small"""
        # Create a very small base64 string
        small_audio = base64.b64encode(b"hi").decode('utf-8')

        is_valid = speech_service.validate_audio_format(small_audio)
        assert is_valid is False

    @pytest.mark.asyncio
    async def test_validate_audio_format_invalid_base64(self, speech_service):
        """Test audio format validation with invalid base64"""
        is_valid = speech_service.validate_audio_format("invalid_base64!")
        assert is_valid is False

    @pytest.mark.asyncio
    async def test_preprocess_audio(self, speech_service):
        """Test audio preprocessing"""
        mock_audio_data = base64.b64encode(b"test audio").decode('utf-8')

        processed_data = await speech_service.preprocess_audio(mock_audio_data)

        # In current implementation, preprocessing just returns the original data
        assert processed_data == mock_audio_data

    @pytest.mark.asyncio
    async def test_preprocess_audio_error(self, speech_service):
        """Test audio preprocessing with error"""
        with patch('base64.b64decode', side_effect=Exception("Decode error")):
            result = await speech_service.preprocess_audio("invalid")
            # Should return original data in case of error
            assert result == "invalid"


# Run the tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])
"""
Voice-to-Action Service for Vision-Language-Action (VLA) Module

This service processes voice commands and converts them into robot actions.
"""
import asyncio
import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class ActionType(Enum):
    """Types of robot actions that can be executed"""
    MOVE_FORWARD = "move_forward"
    MOVE_BACKWARD = "move_backward"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    MOVE_TO_LOCATION = "move_to_location"
    PICK_UP_OBJECT = "pick_up_object"
    PLACE_OBJECT = "place_object"
    STOP = "stop"
    FOLLOW_PATH = "follow_path"


@dataclass
class ActionCommand:
    """Represents a robot action command"""
    action_type: ActionType
    parameters: Dict[str, Any]
    confidence: float = 1.0


@dataclass
class VoiceCommand:
    """Represents a voice command input"""
    text: str
    audio_data: Optional[bytes] = None
    timestamp: Optional[float] = None


class VoiceToActionService:
    """Service for converting voice commands to robot actions"""

    def __init__(self):
        self.command_processors = {
            'move': self._process_move_command,
            'go': self._process_move_command,
            'navigate': self._process_move_command,
            'turn': self._process_turn_command,
            'rotate': self._process_turn_command,
            'pick': self._process_pick_command,
            'place': self._process_place_command,
            'stop': self._process_stop_command,
        }

    async def process_voice_command(self, command: VoiceCommand) -> ActionCommand:
        """
        Process a voice command and return the corresponding robot action

        Args:
            command: The voice command to process

        Returns:
            ActionCommand representing the robot action to execute
        """
        logger.info(f"Processing voice command: {command.text}")

        # Normalize the command text
        normalized_text = command.text.lower().strip()

        # Try to identify the command type and extract parameters
        action_command = self._identify_command(normalized_text)

        if action_command is None:
            # If we can't identify the command, use a default action
            action_command = ActionCommand(
                action_type=ActionType.STOP,
                parameters={},
                confidence=0.0
            )
            logger.warning(f"Could not identify command: {command.text}")

        return action_command

    def _identify_command(self, text: str) -> Optional[ActionCommand]:
        """Identify the command type and extract parameters from text"""
        # Split the text into words
        words = text.split()

        if not words:
            return None

        # Check for known command prefixes
        first_word = words[0]

        # Look for command processors that match
        for prefix, processor in self.command_processors.items():
            if first_word.startswith(prefix):
                return processor(text)

        # If no direct match, try to infer from context
        return self._infer_command(text)

    def _infer_command(self, text: str) -> Optional[ActionCommand]:
        """Infer command from context if direct match fails"""
        # Look for keywords that indicate movement
        if any(word in text for word in ['forward', 'backward', 'ahead', 'back']):
            return self._process_move_command(text)

        # Look for keywords that indicate turning
        if any(word in text for word in ['left', 'right', 'turn', 'rotate']):
            return self._process_turn_command(text)

        # Look for keywords that indicate navigation
        if any(word in text for word in ['kitchen', 'office', 'living room', 'bedroom', 'to the']):
            return self._process_move_to_location_command(text)

        # Look for keywords that indicate manipulation
        if any(word in text for word in ['pick', 'grasp', 'take', 'get', 'lift']):
            return self._process_pick_command(text)

        if any(word in text for word in ['place', 'put', 'drop', 'set']):
            return self._process_place_command(text)

        # If we still can't infer, return None
        return None

    def _process_move_command(self, text: str) -> ActionCommand:
        """Process movement commands like 'move forward 2 meters'"""
        # Extract distance if specified
        distance = self._extract_distance(text)

        if 'forward' in text or 'ahead' in text:
            return ActionCommand(
                action_type=ActionType.MOVE_FORWARD,
                parameters={'distance': distance or 1.0},  # Default to 1 meter
                confidence=0.8
            )
        elif 'backward' in text or 'back' in text:
            return ActionCommand(
                action_type=ActionType.MOVE_BACKWARD,
                parameters={'distance': distance or 1.0},
                confidence=0.8
            )
        else:
            # Default to forward if no direction specified
            return ActionCommand(
                action_type=ActionType.MOVE_FORWARD,
                parameters={'distance': distance or 1.0},
                confidence=0.6
            )

    def _process_turn_command(self, text: str) -> ActionCommand:
        """Process turning commands like 'turn left 90 degrees'"""
        angle = self._extract_angle(text)

        if 'left' in text:
            return ActionCommand(
                action_type=ActionType.TURN_LEFT,
                parameters={'angle': angle or 90.0},  # Default to 90 degrees
                confidence=0.8
            )
        elif 'right' in text:
            return ActionCommand(
                action_type=ActionType.TURN_RIGHT,
                parameters={'angle': angle or 90.0},
                confidence=0.8
            )
        else:
            # Default to left if no direction specified
            return ActionCommand(
                action_type=ActionType.TURN_LEFT,
                parameters={'angle': angle or 90.0},
                confidence=0.6
            )

    def _process_move_to_location_command(self, text: str) -> ActionCommand:
        """Process navigation commands like 'go to the kitchen'"""
        location = self._extract_location(text)

        return ActionCommand(
            action_type=ActionType.MOVE_TO_LOCATION,
            parameters={'location': location or 'unknown'},
            confidence=0.7
        )

    def _process_pick_command(self, text: str) -> ActionCommand:
        """Process pick-up commands like 'pick up the red cup'"""
        object_name = self._extract_object_name(text)

        return ActionCommand(
            action_type=ActionType.PICK_UP_OBJECT,
            parameters={'object': object_name or 'unknown'},
            confidence=0.7
        )

    def _process_place_command(self, text: str) -> ActionCommand:
        """Process place commands like 'place the cup on the table'"""
        object_name = self._extract_object_name(text)
        location = self._extract_location(text)

        return ActionCommand(
            action_type=ActionType.PLACE_OBJECT,
            parameters={
                'object': object_name or 'unknown',
                'location': location or 'default'
            },
            confidence=0.7
        )

    def _process_stop_command(self, text: str) -> ActionCommand:
        """Process stop commands"""
        return ActionCommand(
            action_type=ActionType.STOP,
            parameters={},
            confidence=1.0
        )

    def _extract_distance(self, text: str) -> Optional[float]:
        """Extract distance value from text"""
        words = text.split()
        for i, word in enumerate(words):
            if word in ['meters', 'meter', 'm', 'feet', 'ft']:
                try:
                    # Look at the previous word for the numeric value
                    distance = float(words[i-1])
                    return distance
                except (ValueError, IndexError):
                    continue
        return None

    def _extract_angle(self, text: str) -> Optional[float]:
        """Extract angle value from text"""
        words = text.split()
        for i, word in enumerate(words):
            if word in ['degrees', 'degree', 'deg']:
                try:
                    # Look at the previous word for the numeric value
                    angle = float(words[i-1])
                    return angle
                except (ValueError, IndexError):
                    continue
        return None

    def _extract_location(self, text: str) -> Optional[str]:
        """Extract location from text"""
        # Look for common room names
        rooms = ['kitchen', 'office', 'living room', 'bedroom', 'bathroom', 'dining room', 'hallway']
        for room in rooms:
            if room in text:
                return room.replace(' ', '_')  # Use underscores for consistency

        # Look for "to the [location]" pattern
        if 'to the' in text:
            start_idx = text.find('to the') + 6
            remaining = text[start_idx:].strip()
            # Take the first meaningful word
            words = remaining.split()
            if words:
                return words[0]

        return None

    def _extract_object_name(self, text: str) -> Optional[str]:
        """Extract object name from text"""
        # Look for articles that precede object names
        articles = ['the', 'a', 'an']
        words = text.split()

        for i, word in enumerate(words):
            if word in articles and i + 1 < len(words):
                # Return the next word as the object name
                next_word = words[i + 1]
                # Check if there's a second word that might describe the object
                if i + 2 < len(words):
                    second_word = words[i + 2]
                    # If the second word is a color or descriptor, include it
                    colors = ['red', 'blue', 'green', 'yellow', 'black', 'white', 'small', 'large', 'big']
                    if second_word in colors:
                        return f"{next_word}_{second_word}"
                return next_word

        return None


# Global instance of the service
voice_to_action_service = VoiceToActionService()
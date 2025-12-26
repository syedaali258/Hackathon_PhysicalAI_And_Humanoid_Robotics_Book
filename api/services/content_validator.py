"""
Content Validation Service for VLA Module Technical Accuracy
"""
import asyncio
import logging
from typing import Dict, List, Any, Optional
from datetime import datetime

logger = logging.getLogger(__name__)


class VLAContentValidator:
    """
    Service for validating VLA module content against technical accuracy standards
    """

    def __init__(self):
        # Define technical terms and concepts specific to VLA module
        self.vla_technical_terms = {
            "VLA", "Vision-Language-Action", "vision system", "language system", "action system",
            "speech recognition", "natural language processing", "robot control", "simulation environment",
            "ROS 2", "NVIDIA Isaac", "Gazebo", "Unity", "LLM", "large language model",
            "perception", "planning", "control", "humanoid robot", "robotics"
        }

        # Define valid command patterns for VLA systems
        self.valid_command_patterns = [
            "navigate", "move", "turn", "grasp", "detect", "find", "pick up", "bring",
            "go to", "locate", "identify", "manipulate", "interact"
        ]

        # Define safety and constraint keywords
        self.safety_keywords = [
            "safety", "constraint", "validation", "check", "limit", "boundary", "safe",
            "hazard", "risk", "secure", "protect", "limitation"
        ]

    async def validate_vla_content(self, content: str, module_id: str = "04-vla-module") -> Dict[str, Any]:
        """
        Validate VLA module content against technical accuracy standards

        Args:
            content: Content to validate
            module_id: Module identifier (should be "04-vla-module")

        Returns:
            Dictionary with validation results
        """
        try:
            issues = []
            suggestions = []

            # Check if this is actually VLA content
            if module_id != "04-vla-module":
                issues.append(f"Content validation requested for wrong module: {module_id}")
                return {
                    "valid": False,
                    "issues": issues,
                    "suggestions": suggestions,
                    "confidence": 0.0
                }

            # Check for VLA-specific terminology
            content_lower = content.lower()
            has_vla_terms = any(term.lower() in content_lower for term in self.vla_technical_terms)
            if not has_vla_terms:
                issues.append("Content appears to be missing key VLA (Vision-Language-Action) terminology")

            # Check for proper structure
            lines = content.split('\n')
            has_headers = any(line.strip().startswith('# ') for line in lines)
            if not has_headers:
                suggestions.append("Consider adding proper markdown headers for better structure")

            # Check content length for comprehensiveness
            if len(content) < 300:
                issues.append("Content appears to be too brief for a comprehensive VLA module topic")

            # Check for technical depth vs conceptual explanation balance
            technical_indicators = ["code", "```", "API", "function", "class", "method", "parameter", "model"]
            conceptual_indicators = ["concept", "understand", "learn", "knowledge", "principle", "theory"]

            has_technical = any(indicator in content_lower for indicator in technical_indicators)
            has_conceptual = any(indicator in content_lower for indicator in conceptual_indicators)

            if not (has_technical or has_conceptual):
                suggestions.append("Consider including both technical implementation and conceptual understanding")

            # Check for command examples if appropriate
            has_commands = any(pattern in content_lower for pattern in self.valid_command_patterns)
            if "command" in content_lower or "action" in content_lower:
                if not has_commands:
                    suggestions.append("Consider including example commands or actions")

            # Check for safety considerations
            has_safety = any(keyword in content_lower for keyword in self.safety_keywords)
            if "robot" in content_lower or "action" in content_lower:
                if not has_safety and "safety" not in content_lower:
                    suggestions.append("Consider addressing safety constraints for robot actions")

            # Check for simulation context if relevant
            if "simulation" in content_lower or "environment" in content_lower:
                if not any(sim_tool in content_lower for sim_tool in ["gazebo", "unity", "isaac", "simulation"]):
                    suggestions.append("Consider specifying the simulation environment being used")

            # Check for ROS 2 integration if relevant
            if "control" in content_lower or "navigation" in content_lower:
                if "ros" not in content_lower and "ROS" not in content_lower:
                    suggestions.append("Consider mentioning ROS 2 integration for robot control")

            # Calculate confidence based on issues
            issue_count = len(issues)
            confidence = max(0.1, 1.0 - (issue_count * 0.15))

            return {
                "valid": len(issues) == 0,
                "issues": issues,
                "suggestions": suggestions,
                "confidence": round(confidence, 2),
                "content_length": len(content),
                "technical_terms_found": [term for term in self.vla_technical_terms if term.lower() in content_lower]
            }

        except Exception as e:
            logger.error(f"Error validating VLA content: {str(e)}")
            return {
                "valid": False,
                "issues": [f"Validation error: {str(e)}"],
                "suggestions": [],
                "confidence": 0.0
            }

    async def validate_vla_code_example(self, code: str, language: str = "python") -> Dict[str, Any]:
        """
        Validate code examples for VLA implementations

        Args:
            code: Code example to validate
            language: Programming language of the code

        Returns:
            Dictionary with validation results
        """
        issues = []
        suggestions = []

        code_lower = code.lower()

        # Check for common VLA patterns in code
        required_imports = {
            "python": ["speech_recognition", "openai", "ros2", "robot", "simulation"]
        }

        if language in required_imports:
            has_relevant_imports = any(imp in code_lower for imp in required_imports[language])
            if not has_relevant_imports:
                suggestions.append(f"Consider importing relevant libraries for {language} VLA implementation")

        # Check for error handling
        if language == "python":
            if "try:" not in code_lower and "except" not in code_lower:
                suggestions.append("Consider adding error handling for robust VLA implementation")

        # Check for async patterns if appropriate
        if any(pattern in code_lower for pattern in ["async", "await"]):
            if "asyncio" not in code_lower:
                suggestions.append("Consider importing asyncio for async operations")

        # Check for simulation integration
        if "robot" in code_lower and "simulation" in code_lower:
            if not any(sim_pattern in code_lower for sim_pattern in ["gazebo", "unity", "isaac", "env", "environment"]):
                suggestions.append("Consider showing simulation environment integration")

        return {
            "valid": len(issues) == 0,
            "issues": issues,
            "suggestions": suggestions,
            "confidence": 0.9 if len(issues) == 0 else max(0.1, 1.0 - len(issues) * 0.2)
        }

    async def validate_vla_command(self, command: str) -> Dict[str, Any]:
        """
        Validate a VLA command for technical feasibility and safety

        Args:
            command: Natural language command to validate

        Returns:
            Dictionary with validation results
        """
        issues = []
        suggestions = []

        command_lower = command.lower().strip()

        if not command_lower:
            issues.append("Command is empty")
            return {
                "valid": False,
                "issues": issues,
                "suggestions": suggestions,
                "confidence": 0.0
            }

        # Check if command contains actionable verbs
        has_action = any(pattern in command_lower for pattern in self.valid_command_patterns)
        if not has_action:
            issues.append(f"Command '{command}' doesn't contain recognizable action patterns")

        # Check for potentially unsafe commands
        unsafe_patterns = ["destroy", "harm", "damage", "break", "injure"]
        has_unsafe = any(pattern in command_lower for pattern in unsafe_patterns)
        if has_unsafe:
            issues.append(f"Command '{command}' contains potentially unsafe actions")

        # Check command length and complexity
        words = command.split()
        if len(words) < 2:
            suggestions.append("Command should have at least a verb and object")

        # Check for specific target or location
        if any(action in command_lower for action in ["navigate", "go to", "move to"]) and len(words) < 3:
            suggestions.append("Navigation commands should specify a target location")

        return {
            "valid": len(issues) == 0,
            "issues": issues,
            "suggestions": suggestions,
            "confidence": 0.95 if len(issues) == 0 else max(0.1, 1.0 - len(issues) * 0.3)
        }

    async def validate_vla_pipeline(self, pipeline_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate a complete VLA pipeline for technical soundness

        Args:
            pipeline_data: Dictionary containing pipeline information

        Returns:
            Dictionary with validation results
        """
        issues = []
        suggestions = []

        # Check required pipeline components
        required_keys = ["voice_input", "language_processing", "action_generation"]
        missing_keys = [key for key in required_keys if key not in pipeline_data]
        if missing_keys:
            issues.extend([f"Missing required pipeline component: {key}" for key in missing_keys])

        # Validate voice input component
        if "voice_input" in pipeline_data:
            voice_input = pipeline_data["voice_input"]
            if not isinstance(voice_input, (str, dict)):
                issues.append("Voice input should be string or structured data")

        # Validate language processing component
        if "language_processing" in pipeline_data:
            lang_proc = pipeline_data["language_processing"]
            if not isinstance(lang_proc, dict):
                issues.append("Language processing should be structured data")

        # Validate action generation component
        if "action_generation" in pipeline_data:
            actions = pipeline_data["action_generation"]
            if not isinstance(actions, list):
                issues.append("Action generation should result in a list of actions")

        # Check for safety validation step
        has_safety_check = pipeline_data.get("safety_validation", False) or \
                          any("safety" in str(val).lower() for val in pipeline_data.values())
        if not has_safety_check:
            suggestions.append("Consider adding safety validation step to pipeline")

        return {
            "valid": len(issues) == 0,
            "issues": issues,
            "suggestions": suggestions,
            "confidence": 0.9 if len(issues) == 0 else max(0.1, 1.0 - len(issues) * 0.2)
        }


# Singleton instance
vla_content_validator = VLAContentValidator()
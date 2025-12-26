"""
Isaac Content Service for the AI/Robotics Book API

This service handles Isaac-specific content retrieval and management
for Module 3: The AI-Robot Brain (NVIDIA Isaac).
"""

from typing import Dict, Any, Optional, List
from datetime import datetime
import uuid
import logging
from api.models.content import ContentPiece
from api.services.content_service import content_service
from api.utils.errors import ContentException


class IsaacContentService:
    """
    Service class for handling Isaac-specific content operations.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)

    async def get_isaac_content_by_type(self, content_type: str, filters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Retrieve Isaac-specific content pieces by type with optional filters.

        Args:
            content_type: Type of Isaac content (isaac-sim, synthetic-data, slam, navigation, etc.)
            filters: Optional filters for content retrieval

        Returns:
            List of Isaac-specific content pieces
        """
        if filters is None:
            filters = {}

        # Define valid Isaac content types
        valid_types = [
            "isaac-sim", "synthetic-data", "slam", "navigation",
            "sensor-simulation", "perception", "isaac-ros",
            "isaac-components", "simulation-environment"
        ]

        if content_type not in valid_types:
            raise ContentException(f"Invalid Isaac content type. Must be one of: {valid_types}")

        # Build search query for Isaac-specific content
        query_parts = [content_type.replace('-', ' ')]

        # Add Isaac-specific tags to the search
        isaac_tags = ["isaac", "nvidia-isaac", "isaac-sim", "isaac-ros"]
        if content_type in ["slam", "navigation", "sensor-simulation"]:
            isaac_tags.append(content_type.replace('-', ' '))

        # Use the base content service to search for Isaac-specific content
        search_filters = {
            "tags": isaac_tags,
            "module": "module-3-ai-brain"
        }
        search_filters.update(filters)

        # Perform the search using the base content service
        results = await content_service.search_content(
            query=" ".join(query_parts),
            filters=search_filters
        )

        # Filter results to ensure they are Isaac-specific
        isaac_results = []
        for result in results:
            # Check if content has Isaac-specific tags or references
            content_metadata = result.get("metadata", {})
            content_tags = content_metadata.get("tags", [])

            # Check if this content is Isaac-specific
            has_isaac_tag = any(tag in content_tags for tag in ["isaac", "nvidia-isaac", "isaac-sim", "isaac-ros"])
            has_isaac_content = any(keyword in result.get("title", "").lower() or keyword in result.get("content", "").lower()
                                   for keyword in ["isaac", "nvidia", "simulation", "synthetic", "slam", "nav2"])

            if has_isaac_tag or has_isaac_content:
                # Add Isaac-specific metadata
                result["isaac_metadata"] = self._enhance_with_isaac_metadata(result)
                isaac_results.append(result)

        self.logger.info(
            f"Retrieved Isaac content",
            extra={
                "content_type": content_type,
                "filter_count": len(filters),
                "result_count": len(isaac_results)
            }
        )

        return isaac_results

    def _enhance_with_isaac_metadata(self, content_piece: Dict[str, Any]) -> Dict[str, Any]:
        """
        Enhance content with Isaac-specific metadata.

        Args:
            content_piece: Base content piece

        Returns:
            Dictionary with Isaac-specific metadata
        """
        isaac_metadata = {
            "is_isaac_specific": True,
            "supported_isaac_versions": [],
            "ros_compatibility": [],
            "simulation_requirements": {},
            "perception_pipeline": False,
            "navigation_pipeline": False,
            "sensor_simulation": False
        }

        # Analyze content to determine Isaac-specific attributes
        content_text = f"{content_piece.get('title', '')} {content_piece.get('content', '')}".lower()

        # Determine Isaac components mentioned in content
        if "slam" in content_text or "visual slam" in content_text or "vslam" in content_text:
            isaac_metadata["perception_pipeline"] = True

        if "navigation" in content_text or "nav2" in content_text or "path planning" in content_text:
            isaac_metadata["navigation_pipeline"] = True

        if "sensor" in content_text or "lidar" in content_text or "camera" in content_text or "imu" in content_text:
            isaac_metadata["sensor_simulation"] = True

        # Determine ROS compatibility based on content
        if "ros 2" in content_text or "ros2" in content_text:
            isaac_metadata["ros_compatibility"] = ["humble", "iron"]

        # Set default supported versions
        isaac_metadata["supported_isaac_versions"] = ["2023.1", "2024.1", "latest"]

        return isaac_metadata

    async def search_isaac_content(self, query: str, filters: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Search for Isaac-specific content with enhanced Isaac-specific ranking.

        Args:
            query: Search query for Isaac content
            filters: Additional filters for Isaac content search

        Returns:
            Dictionary containing search results with Isaac-specific enhancements
        """
        if filters is None:
            filters = {}

        # Add Isaac-specific filters to ensure we're searching Isaac content
        isaac_filters = {
            "module": "module-3-ai-brain",
            "tags": ["isaac", "nvidia-isaac", "isaac-sim", "isaac-ros"]
        }
        isaac_filters.update(filters)

        # Use base content service for search
        base_results = await content_service.search_content(
            query=query,
            filters=isaac_filters
        )

        # Enhance results with Isaac-specific information
        enhanced_results = []
        for result in base_results:
            enhanced_result = result.copy()

            # Add Isaac-specific metadata
            enhanced_result["isaac_metadata"] = self._enhance_with_isaac_metadata(result)

            # Calculate Isaac-specific relevance score
            isaac_relevance = self._calculate_isaac_relevance(query.lower(), result)
            enhanced_result["isaac_relevance_score"] = isaac_relevance

            enhanced_results.append(enhanced_result)

        # Sort by Isaac-specific relevance
        enhanced_results.sort(key=lambda x: x["isaac_relevance_score"], reverse=True)

        search_result = {
            "results": enhanced_results,
            "query": query,
            "total": len(enhanced_results),
            "isaac_specific": True,
            "enhancement_applied": True
        }

        self.logger.info(
            f"Searched Isaac content",
            extra={
                "query": query,
                "result_count": len(enhanced_results),
                "filter_count": len(filters)
            }
        )

        return search_result

    def _calculate_isaac_relevance(self, query: str, content: Dict[str, Any]) -> float:
        """
        Calculate Isaac-specific relevance score for content.

        Args:
            query: Search query
            content: Content piece to evaluate

        Returns:
            Float between 0 and 1 representing Isaac-specific relevance
        """
        relevance_score = 0.0

        # Check for Isaac-specific keywords in title and content
        content_text = f"{content.get('title', '')} {content.get('content', '')}".lower()

        # Weighted keywords for Isaac relevance
        isaac_keywords = [
            ("isaac", 0.3), ("nvidia", 0.2), ("simulation", 0.2),
            ("synthetic", 0.25), ("slam", 0.3), ("nav2", 0.3),
            ("navigation", 0.2), ("sensor", 0.15), ("perception", 0.25),
            ("isaac sim", 0.4), ("isaac ros", 0.4), ("visual slam", 0.35)
        ]

        for keyword, weight in isaac_keywords:
            if keyword in content_text:
                relevance_score += weight
            if keyword in query:
                relevance_score += weight * 0.5  # Additional boost if in query

        # Check for Isaac-specific concepts
        if any(concept in content_text for concept in ["domain randomization", "gym environment", "physics simulation", "sensor model"]):
            relevance_score += 0.2

        # Normalize to 0-1 range
        return min(relevance_score, 1.0)

    async def get_isaac_component_examples(self, component_type: str) -> List[Dict[str, Any]]:
        """
        Get Isaac-specific component examples for educational purposes.

        Args:
            component_type: Type of Isaac component (e.g., "slam", "navigation", "sensor")

        Returns:
            List of Isaac component examples with educational value
        """
        valid_components = ["slam", "navigation", "sensor", "simulation", "perception"]
        if component_type not in valid_components:
            raise ContentException(f"Invalid Isaac component type. Must be one of: {valid_components}")

        # Return Isaac-specific examples based on component type
        examples = {
            "slam": [
                {
                    "id": "isaac-slam-example-1",
                    "title": "Visual SLAM with Isaac ROS VIO",
                    "description": "Example of Visual-Inertial Odometry implementation using Isaac ROS packages",
                    "code": '''# Isaac ROS VIO Example
from isaac_ros_visual_slam import VisualSlamNode
import rclpy

def main():
    rclpy.init()
    visual_slam_node = VisualSlamNode()

    # Configure parameters
    visual_slam_node.set_parameters([
        Parameter('enable_rectification', True),
        Parameter('publish_odom_tf', True)
    ])

    rclpy.spin(visual_slam_node)
    visual_slam_node.destroy_node()
    rclpy.shutdown()''',
                    "explanation": "This example demonstrates how to use Isaac ROS Visual Slam package to perform visual-inertial odometry for humanoid robots.",
                    "simulation_env": "isaac_sim",
                    "complexity": "intermediate"
                }
            ],
            "navigation": [
                {
                    "id": "isaac-nav-example-1",
                    "title": "Nav2 with Isaac Sim Integration",
                    "description": "Example of Nav2 navigation stack integrated with Isaac Sim",
                    "code": '''# Nav2 Isaac Integration Example
from nav2_simple_commander import BasicNavigator
import rclpy

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    # ... set pose

    navigator.setInitialPose(initial_pose)

    # Navigate to goal
    goal_pose = PoseStamped()
    # ... set goal pose

    navigator.goToPose(goal_pose)''',
                    "explanation": "This example shows how to integrate Nav2 navigation with Isaac Sim for testing navigation algorithms in simulation.",
                    "simulation_env": "isaac_sim",
                    "complexity": "advanced"
                }
            ],
            "sensor": [
                {
                    "id": "isaac-sensor-example-1",
                    "title": "Isaac Sim Sensor Simulation",
                    "description": "Example of configuring realistic sensor models in Isaac Sim",
                    "code": '''# Isaac Sim Sensor Configuration
from omni.isaac.core import World
from omni.isaac.sensor import Camera

# Create world
world = World(stage_units_in_meters=1.0)

# Add camera sensor
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([0.5, 0.0, 0.5]),
    frequency=30,
    resolution=(640, 480)
)

# Enable various sensor outputs
camera.add_ground_truth_to_frame("depth", "distance_to_image_plane")
camera.add_ground_truth_to_frame("semantic_segmentation", "semantic_segmentation")''',
                    "explanation": "This example demonstrates how to configure realistic sensor models in Isaac Sim with ground truth annotations.",
                    "simulation_env": "isaac_sim",
                    "complexity": "intermediate"
                }
            ]
        }

        component_examples = examples.get(component_type, [])

        self.logger.info(
            f"Retrieved Isaac component examples",
            extra={
                "component_type": component_type,
                "example_count": len(component_examples)
            }
        )

        return component_examples


# Global instance of the service
isaac_content_service = IsaacContentService()
# Data Model: AI/Spec-Driven Book using Docusaurus - Module 3: The AI-Robot Brain (NVIDIA Isaac)

**Feature**: Module 3: The AI-Robot Brain (NVIDIA Isaac)
**Date**: 2025-12-22
**Modeler**: Claude Code

## Overview

This document defines the data models for the AI-native book on Physical AI & Humanoid Robotics with embedded RAG chatbot, specifically for Module 3: The AI-Robot Brain (NVIDIA Isaac). The models support content management, simulation visualization, user interactions, and RAG functionality while maintaining zero hallucination constraints and technical accuracy for Isaac/ROS-specific content.

## Core Entities

### 1. BookContent

**Description**: Represents the content of the book including modules, chapters, and sections with Isaac/ROS-specific attributes

**Fields**:
- `id` (string): Unique identifier for the content piece
- `type` (enum): Content type (module, chapter, section, example, exercise, simulation, isaac-component)
- `title` (string): Title of the content piece
- `slug` (string): URL-friendly identifier
- `content` (string): Main content in Markdown/MDX format
- `metadata` (object): Additional content metadata
  - `author` (string): Content author
  - `lastUpdated` (date): Last modification date
  - `version` (string): Content version
  - `tags` (array): Content tags for categorization
  - `prerequisites` (array): Prerequisite content IDs
  - `isaacComponents` (array): Isaac-specific components referenced in content
  - `rosPackages` (array): ROS 2 packages referenced in content
  - `simulationRequirements` (object): Simulation-specific requirements
    - `isaacVersion` (string): Required Isaac version
    - `rosDistribution` (string): Required ROS 2 distribution
    - `simulationEnvironment` (string): Required simulation environment
- `hierarchy` (object): Position in the book structure
  - `moduleNumber` (integer): Module sequence number
  - `chapterNumber` (integer): Chapter sequence number within module
  - `sectionNumber` (integer): Section sequence number within chapter
- `references` (array): Official documentation references for validation

**Relationships**:
- Parent: One parent content piece (except modules)
- Children: Multiple child content pieces
- Prerequisites: Multiple prerequisite content pieces
- IsaacsComponents: Multiple Isaac-specific components referenced

**Validation Rules**:
- Title must be 3-100 characters
- Slug must be URL-friendly (alphanumeric, hyphens, underscores)
- Content must be in valid Markdown/MDX format
- Prerequisites must exist in the system
- Isaac components must reference valid Isaac documentation
- ROS packages must reference valid ROS 2 documentation

### 2. UserSession

**Description**: Represents a user's interaction session with the book and chatbot, with Isaac/ROS-specific progress tracking

**Fields**:
- `id` (string): Unique session identifier
- `userId` (string): User identifier (anonymous or authenticated)
- `startTime` (datetime): Session start time
- `endTime` (datetime): Session end time (null if active)
- `currentLocation` (object): Current position in the book
  - `moduleId` (string): Current module ID
  - `chapterId` (string): Current chapter ID
  - `sectionId` (string): Current section ID
- `progress` (array): User progress tracking
  - `contentId` (string): Content piece ID
  - `completionStatus` (enum): not_started, in_progress, completed
  - `timeSpent` (integer): Time spent in seconds
  - `lastAccessed` (datetime): Last access time
- `isaacProgress` (array): Isaac-specific progress - Module 3 specific
  - `isaacComponentId` (string): ID of the Isaac component
  - `completionStatus` (enum): not_started, in_progress, completed
  - `attempts` (integer): Number of attempts at Isaac-specific exercises
  - `simulationEnvironment` (string): Simulation environment used
  - `validationResults` (object): Results of Isaac-specific validations

**Relationships**:
- User: Associated user (if authenticated)
- CurrentContent: Currently viewed content piece
- Progress: Multiple content progress records
- IsaacProgress: Multiple Isaac-specific progress records

**Validation Rules**:
- Session must have a start time
- End time must be after start time (if set)
- Current location must reference valid content
- Isaac progress must reference valid Isaac components

### 3. ChatMessage

**Description**: Represents a message in the RAG-enabled chatbot conversation with Isaac/ROS-specific context

**Fields**:
- `id` (string): Unique message identifier
- `sessionId` (string): Associated user session ID
- `timestamp` (datetime): Message timestamp
- `sender` (enum): sender type (user, assistant)
- `content` (string): Message content
- `sourceChunks` (array): Source content chunks used for response
  - `chunkId` (string): Content chunk identifier
  - `text` (string): Chunk text content
  - `relevanceScore` (number): Relevance score (0-1)
  - `contentId` (string): Original content piece ID
- `confidence` (number): Confidence score of the response (0-1)
- `followUpSuggestions` (array): Suggested follow-up questions
- `isaacContext` (object): Context related to Isaac/ROS content - Module 3 specific
  - `currentIsaacComponent` (string): Currently referenced Isaac component
  - `simulationEnvironment` (string): Isaac simulation environment
  - `rosPackage` (string): Referenced ROS 2 package
  - `isaacConcepts` (array): Isaac concepts covered in the response

**Relationships**:
- Session: Associated user session
- SourceContent: Multiple source content pieces used in response

**Validation Rules**:
- Content must not exceed 10,000 characters
- Confidence score must be between 0 and 1
- Source chunks must reference valid content pieces
- Confidence must be high (>0.7) for responses
- Isaac context must reference valid Isaac components

### 4. ContentChunk

**Description**: Represents a chunk of content used for RAG retrieval with Isaac/ROS-specific metadata

**Fields**:
- `id` (string): Unique chunk identifier
- `contentId` (string): Original content piece ID
- `text` (string): Chunk text content
- `embedding` (array): Vector embedding for similarity search
- `chunkIndex` (integer): Position of chunk in original content
- `metadata` (object): Additional metadata
  - `contentType` (string): Type of content in chunk
  - `keywords` (array): Keywords extracted from chunk
  - `summary` (string): Brief summary of chunk content
  - `isaacRelevance` (boolean): Whether chunk relates to Isaac content - Module 3 specific
  - `isaacComponentType` (string): Isaac component type (simulator, perception, navigation)
  - `rosPackage` (string): ROS 2 package referenced in chunk
  - `simulationConcepts` (array): Simulation concepts covered in chunk

**Relationships**:
- OriginalContent: Source content piece
- ChatMessages: Multiple chat messages that reference this chunk

**Validation Rules**:
- Text must be between 50 and 1000 characters
- Embedding must have consistent dimensions
- Chunk index must be non-negative
- Isaac relevance must be boolean
- Isaac component type must be one of: simulator, perception, navigation, none

### 5. IsaacSimulationComponent

**Description**: Represents Isaac-specific simulation components for Module 3 content

**Fields**:
- `id` (string): Unique identifier for the Isaac simulation component
- `name` (string): Name of the Isaac simulation component
- `type` (enum): Type of Isaac component (simulator, perception, navigation, sensor)
- `description` (string): Description of the Isaac component
- `contentId` (string): Associated content piece ID
- `isaacMetadata` (object): Isaac-specific metadata
  - `packageName` (string): Isaac package name (e.g., Isaac Sim, Isaac ROS)
  - `version` (string): Required Isaac version
  - `dependencies` (array): Other Isaac packages this component depends on
  - `simulationType` (enum): synthetic-data, slam, navigation, sensor-modeling
- `parameters` (object): Configuration parameters for the simulation
  - `simulationEnvironment` (string): Isaac simulation environment
  - `sensorModels` (array): Types of sensors simulated (camera, lidar, etc.)
  - `evaluationMetrics` (object): Metrics for evaluating component performance
- `metadata` (object): Additional metadata
  - `difficultyLevel` (enum): beginner, intermediate, advanced
  - `estimatedTime` (integer): Estimated time to complete in minutes
  - `learningObjectives` (array): Educational objectives of the component
- `createdAt` (datetime): Creation timestamp
- `updatedAt` (datetime): Last update timestamp

**Relationships**:
- Content: Associated book content piece
- UserSessions: Multiple user sessions that interacted with this component

**Validation Rules**:
- Name must be 3-100 characters
- Type must be one of: simulator, perception, navigation, sensor
- Difficulty level must be one of: beginner, intermediate, advanced
- Estimated time must be positive
- Parameters must be valid for the specified Isaac component type
- Simulation type must be one of: synthetic-data, slam, navigation, sensor-modeling

### 6. ModuleProgress

**Description**: Tracks user progress through specific modules with Isaac/ROS-specific metrics

**Fields**:
- `id` (string): Unique progress record identifier
- `userId` (string): User identifier
- `moduleId` (string): Module being tracked
- `completionPercentage` (number): Percentage of module completed (0-100)
- `timeSpent` (integer): Total time spent on module in seconds
- `lastAccessed` (datetime): Last time module was accessed
- `completedChapters` (array): IDs of completed chapters
- `isaacActivitiesCompleted` (integer): Count of completed Isaac activities - Module 3 specific
- `isaacConceptsMastered` (array): Isaac concepts the user has mastered
- `rosPackagesExplored` (array): ROS 2 packages the user has explored
- `quizScores` (array): Scores for module quizzes
  - `quizId` (string): Quiz identifier
  - `score` (number): Score achieved (0-100)
  - `attempts` (integer): Number of attempts

**Relationships**:
- User: Associated user
- Module: Tracked module
- CompletedChapters: Multiple completed chapters

**Validation Rules**:
- Completion percentage must be between 0 and 100
- Time spent must be non-negative
- Quiz scores must be between 0 and 100
- Isaac activities completed must be non-negative
- Isaac concepts mastered must reference valid Isaac concepts
- ROS packages explored must reference valid ROS 2 packages

## State Transitions

### UserSession States
1. **Active**: Session is currently in progress
2. **Paused**: Session temporarily inactive
3. **Completed**: Session has ended

Transition Rules:
- Active → Paused: User navigates away for >5 minutes
- Active → Completed: User explicitly ends session or is inactive for >30 minutes
- Paused → Active: User returns to session within timeout period

### Content Validation States
1. **Draft**: Content is being created
2. **UnderReview**: Content is being validated
3. **Validated**: Content has passed accuracy checks
4. **Published**: Content is available to users

Transition Rules:
- Draft → UnderReview: Content is submitted for validation
- UnderReview → Validated: Content passes accuracy checks against official sources
- UnderReview → Draft: Content requires revisions
- Validated → Published: Content is approved for publication
- Published → UnderReview: Content is updated and needs re-validation

## Relationships Summary

```
BookContent (1) ← → (0..n) BookContent (Parent-Child)
BookContent (1) ← → (0..n) ContentChunk (Content-Chunks)
BookContent (1) ← → (0..n) IsaacSimulationComponent (Content-Isaac)
UserSession (1) ← → (0..n) ChatMessage (Session-Messages)
UserSession (1) ← → (0..n) ModuleProgress (User-Progress)
IsaacSimulationComponent (1) ← → (0..n) UserSession (Isaac-Session)
ChatMessage (0..n) ← → (0..n) ContentChunk (Message-Sources)
ModuleProgress (1) ← → (0..n) IsaacSimulationComponent (Progress-Isaac)
```

## Indexes for Performance

1. **BookContent**: Index on (type, moduleNumber, chapterNumber, sectionNumber, isaacRelevance) for navigation and Isaac-specific filtering
2. **ChatMessage**: Index on (sessionId, timestamp) for conversation retrieval
3. **ContentChunk**: Index on (contentId, chunkIndex, isaacRelevance) for content reconstruction and Isaac-specific retrieval
4. **UserSession**: Index on (userId, startTime) for session tracking
5. **ModuleProgress**: Index on (userId, moduleId) for progress tracking
6. **IsaacSimulationComponent**: Index on (type, difficultyLevel, contentId) for Isaac component retrieval
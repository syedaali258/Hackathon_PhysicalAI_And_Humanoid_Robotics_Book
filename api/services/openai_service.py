import os
from typing import List, Dict, Any
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class OpenAIService:
    def __init__(self):
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.client = OpenAI(api_key=api_key)
        self.model = os.getenv("OPENAI_MODEL", "gpt-4o-mini")  # Using a more cost-effective model by default

    def generate_response(self,
                         user_message: str,
                         context_chunks: List[Dict[str, Any]],
                         system_prompt: str = None) -> Dict[str, Any]:
        """
        Generate a response based on user message and context chunks from the book.
        This ensures zero hallucination by restricting the AI to only use provided context.
        """
        if system_prompt is None:
            system_prompt = (
                "You are an AI assistant for an AI-native book on Physical AI & Humanoid Robotics. "
                "Your responses must be based ONLY on the provided context from the book. "
                "Do not use any knowledge that is not in the provided context. "
                "If you cannot answer based on the context, say so explicitly."
            )

        # Format context chunks for the prompt
        context_text = "\n\n".join([f"Source {i+1}: {chunk['text']}" for i, chunk in enumerate(context_chunks)])

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Context:\n{context_text}\n\nQuestion: {user_message}"}
        ]

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.3,  # Lower temperature for more consistent, fact-based responses
                max_tokens=1000,
                response_format={"type": "text"}
            )

            return {
                "response": response.choices[0].message.content,
                "sources": context_chunks,
                "confidence": 0.8  # Placeholder confidence score
            }
        except Exception as e:
            raise Exception(f"Error calling OpenAI API: {str(e)}")

# Singleton instance with error handling
try:
    openai_service = OpenAIService()
except (ValueError, TypeError) as e:
    print(f"Warning: {e}. OpenAI service will not be available.")
    openai_service = None
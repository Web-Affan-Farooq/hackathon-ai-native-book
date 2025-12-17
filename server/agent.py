import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from agents import AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig, Agent, Runner, function_tool
import asyncio

load_dotenv()

# Get API keys from environment variables
api_key = os.getenv("GEMINI_API_KEY")
cohere_key = os.getenv("COHERE_API_KEY")

# Validate that required environment variables are set
if not api_key:
    raise ValueError("GEMINI_API_KEY environment variable is required")
if not cohere_key:
    raise ValueError("COHERE_API_KEY environment variable is required")

client = AsyncOpenAI(
    api_key=api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=client
)

config = RunConfig(
    model=model,
    model_provider=client,
    tracing_disabled=True,
)


@function_tool
def retrieve_context(query: str) -> str:
    """Retrieve relevant context from the Physical AI and Humanoid Robotics textbook using RAG"""
    try:
        # Validate required environment variables
        qdrant_endpoint = os.getenv("QDRANT_ENDPOINT")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_endpoint or not qdrant_api_key:
            return "RAG system is not properly configured. Missing QDRANT_ENDPOINT or QDRANT_API_KEY."

        cohere_client = cohere.Client(cohere_key)

        qdrant_client = QdrantClient(
            url=qdrant_endpoint,
            api_key=qdrant_api_key,
            timeout=60
        )

        # Get embedding for the query
        response = cohere_client.embed(
            model="embed-english-v3.0",
            input_type="search_query",  # Use search_query for queries
            texts=[query],
        )

        # Search in Qdrant for relevant chunks
        search_results = qdrant_client.query_points(
            collection_name="physical-ai-humanoid-robotics",
            query=response.embeddings[0],
            limit=5
        )

        # Extract the text content from results
        context_texts = []
        for point in search_results.points:
            if "text" in point.payload:
                context_texts.append(point.payload["text"])

        if not context_texts:
            return "No relevant context found in the textbook."

        return "\n\n".join(context_texts[:3])  # Return top 3 most relevant chunks
    except Exception as e:
        print(f"Error retrieving context: {e}")
        return "No relevant context found in the textbook. The RAG system may not be properly configured."


# Define the agent with instructions for the Physical AI and Humanoid Robotics textbook
bot = Agent(
    name="Physical AI and Humanoid Robotics Chatbot",
    instructions="""You are an educational assistant for the Physical AI and Humanoid Robotics textbook.
    Your role is to help students understand robotics concepts by providing clear explanations,
    examples, and guidance based on their experience level. Always be helpful, accurate, and
    educational. When possible, reference content from the textbook by using the retrieve_context tool.
    Adapt your responses to the student's declared experience level (beginner, intermediate, advanced).""",
    tools=[retrieve_context]
)

async def run_agent(question: str):
    try:
        response = await Runner.run(
            starting_agent=bot,
            input=question,
            run_config=config
        )
        print(response.final_output)
    except Exception as e:
        print(f"Error running agent: {e}")
        return f"I encountered an issue processing your request. Please try again. Error: {str(e)}"
    
# asyncio.run(run_agent("tell me what are the basic hardware requirements to get started with this book "))
print("Gemini : ",api_key)
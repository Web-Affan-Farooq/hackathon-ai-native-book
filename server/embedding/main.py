import trafilatura
import os
from dotenv import load_dotenv
import requests 
import xml.etree.ElementTree as ET
from qdrant_client.models import PointStruct
from qdrant_client import QdrantClient
import cohere


class Embedding:
    def __init__(self):
        load_dotenv()
        self.Qclient = QdrantClient(
            url=os.getenv("QDRANT_ENDPOINT"),
            api_key=os.getenv("QDRANT_API_KEY")
        )

        self.cohere = cohere.Client(os.getenv("COHERE_API_KEY"))
        self.EMBED_MODEL = "embed-english-v3.0"
        self.sitemap = os.getenv("SITEMAP_URL")


    def get_all_urls(self):
        xml = requests.get(self.sitemap, timeout=10).text
        root = ET.fromstring(xml)

        urls = []
        for child in root:
            loc = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
            if loc is not None:
                urls.append(loc.text)

        return urls
    
    def extract_text_from_url(self, url):
        html = requests.get(url).text
        text = trafilatura.extract(html)

        if not text:
            print("[WARNING] No text extracted from:", url)

        return text
    
    def chunk_text( self, text, max_chars=1200):
        chunks = []
        while len(text) > max_chars:
            split_pos = text[:max_chars].rfind(". ")
            if split_pos == -1:
                split_pos = max_chars
            chunks.append(text[:split_pos])
            text = text[split_pos:]
        chunks.append(text)
        return chunks
    
    def embed(self, text):
        res = self.cohere.embed(
            model=self.EMBED_MODEL,
            input_type="search_document",
            texts=[text],
        )
        return res.embeddings[0]

    def save_chunk_to_qdrant(self,chunk, chunk_id, url):
        vector = self.embed(chunk)
    
        self.Qclient.upsert(
            collection_name="physical-ai-humanoid-robotics",
            points=[
                PointStruct(
                    id=chunk_id,
                    vector=vector,
                    payload={
                        "url": url,
                        "text": chunk,
                        "chunk_id": chunk_id
                    }
                )
            ]
        )
    def create_rag(self):
        urls = self.get_all_urls()

        global_id = 1

        for url in urls:
            print("\started :", url)
            text = self.extract_text_from_url(url)

            if not text:
                continue

            chunks = self.chunk_text(text)

            for ch in chunks:
                self.save_chunk_to_qdrant(ch, global_id, url)
                print(f"Saved chunk {global_id}")
                global_id += 1

        print("\n✔️ Ingestion completed!")
        print("Total chunks stored:", global_id - 1)
import os
import time
import requests
import trafilatura
import xml.etree.ElementTree as ET
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct


class Embedding:
    def __init__(self):
        load_dotenv()

        self.Qclient = QdrantClient(
            url=os.getenv("QDRANT_ENDPOINT"),
            api_key=os.getenv("QDRANT_API_KEY"),
            timeout=60
        )

        self.cohere = cohere.Client(os.getenv("COHERE_API_KEY"))

        self.EMBED_MODEL = "embed-english-v3.0"
        self.COLLECTION = "physical-ai-humanoid-robotics"
        self.sitemap = os.getenv("SITEMAP_URL")

        self.BATCH_SIZE = 20
        self.MAX_CHARS = 1200
        self.OVERLAP = 200

    # --------------------------------------------------
    # Sitemap
    # --------------------------------------------------
    def get_all_urls(self):
        xml = requests.get(self.sitemap, timeout=15).text
        root = ET.fromstring(xml)

        urls = []
        for child in root:
            loc = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
            if loc is not None:
                urls.append(loc.text)

        return urls

    # --------------------------------------------------
    # Text extraction
    # --------------------------------------------------
    def extract_text_from_url(self, url):
        html = requests.get(url, timeout=15).text
        text = trafilatura.extract(html)

        if not text:
            print(f"[WARNING] No text extracted from {url}")

        return text or ""

    # --------------------------------------------------
    # Sliding window chunking (infinite-loop proof)
    # --------------------------------------------------
    def chunk_text(self, text):
        chunks = []
        text_len = len(text)
        start = 0

        while start < text_len:
            end = min(start + self.MAX_CHARS, text_len)

            if end < text_len:
                split_pos = text.rfind(". ", start, end)
                if split_pos != -1 and split_pos > start:
                    end = split_pos + 2

            chunk = text[start:end].strip()
            if chunk:
                chunks.append(chunk)

            next_start = end - self.OVERLAP
            if next_start <= start:
                next_start = end

            start = next_start

        return chunks

    # --------------------------------------------------
    # Embedding
    # --------------------------------------------------
    def embed_batch(self, texts):
        res = self.cohere.embed(
            model=self.EMBED_MODEL,
            input_type="search_document",
            texts=texts
        )
        return res.embeddings

    # --------------------------------------------------
    # Qdrant upsert with retry
    # --------------------------------------------------
    def upsert_with_retry(self, points, retries=3):
        for attempt in range(retries):
            try:
                self.Qclient.upsert(
                    collection_name=self.COLLECTION,
                    points=points
                )
                return
            except Exception as e:
                print(f"⚠️ Qdrant retry {attempt + 1}: {e}")
                time.sleep(2)

        raise RuntimeError("❌ Failed to upsert points after retries")

    # --------------------------------------------------
    # Main ingestion pipeline
    # --------------------------------------------------
    def create_rag(self):
        print("\n🚀 Starting RAG ingestion\n")

        urls = self.get_all_urls()
        print(f"Found {len(urls)} URLs\n")

        global_id = 1

        for url in urls:
            print(f"🔗 Processing: {url}")

            text = self.extract_text_from_url(url)
            if not text:
                continue

            chunks = self.chunk_text(text)
            print(f"   → {len(chunks)} chunks")

            batch_texts = []
            batch_meta = []

            for ch in chunks:
                batch_texts.append(ch)
                batch_meta.append((global_id, ch))
                global_id += 1

                if len(batch_texts) >= self.BATCH_SIZE:
                    self._flush_batch(batch_texts, batch_meta, url)
                    batch_texts.clear()
                    batch_meta.clear()

            if batch_texts:
                self._flush_batch(batch_texts, batch_meta, url)

        print("\n✅ Ingestion completed successfully")
        print("Total chunks stored:", global_id - 1)

    # --------------------------------------------------
    # Flush helper
    # --------------------------------------------------
    def _flush_batch(self, texts, meta, url):
        vectors = self.embed_batch(texts)

        points = []
        for (chunk_id, chunk), vector in zip(meta, vectors):
            points.append(
                PointStruct(
                    id=chunk_id,
                    vector=vector,
                    payload={
                        "url": url,
                        "chunk_id": chunk_id,
                        "text": chunk
                    }
                )
            )

        self.upsert_with_retry(points)
        print(f"   ✔️ Upserted {len(points)} chunks")
import "./sidenav.css";
import { useEffect, useState } from "react";
import { useSessions } from "../stores/session";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

const Sidenav = () => {
  const { sessions, setSession, setSelectedSession, selectedSession } = useSessions();
  const {siteConfig:{
    customFields
  }} = useDocusaurusContext()
  const [open, setOpen] = useState(false);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchSessions = async () => {
      try {
        setLoading(true);
        setError(null);

        const response = await fetch(`${customFields.api_url}/get-sessions`);

        if (response.status === 404) {
          return window.location.href = `${customFields.client_url}/signup`;
        } else if (response.status === 401) {
          return window.location.href = `${customFields.client_url}/login`;
        } else if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }

        const data = await response.json();
        if (data.sessions) {
          // Map the server response to match our Session type
          const mappedSessions = data.sessions.map((session: any) => ({
            id: session.id,
            name: session.name || `Session ${new Date(session.created_at).toLocaleDateString()}`
          }));

          setSession(mappedSessions);
        }
      } catch (err) {
        console.error("Error fetching sessions:", err);
        setError("Failed to load sessions");
      } finally {
        setLoading(false);
      }
    };

    fetchSessions();
  }, [setSession]);

  const handleSessionClick = (session: any) => {
    setSelectedSession(session);
    setOpen(false); // Close the sidenav when a session is selected
  };

  const toggleSidenav = () => {
    setOpen(!open);
  };

  return (
    <>
      {/* Toggle button */}
      <button
        onClick={toggleSidenav}
        style={{
          position: 'fixed',
          right: open ? '300px' : '20px',
          top: '20px',
          zIndex: 101,
          padding: '10px 15px',
          backgroundColor: '#007cba',
          color: 'white',
          border: 'none',
          borderRadius: '5px',
          cursor: 'pointer',
          fontSize: '14px'
        }}
      >
        {open ? 'Close' : 'Sessions'}
      </button>

      {/* Sidenav panel */}
      <div
        className="sidebar"
        style={{
          transform: open ? 'translateX(0)' : 'translateX(100%)',
          transition: 'transform 0.3s ease-in-out',
          width: '300px',
          height: '500px',
          borderRadius: '20px 0 0 20px',
          backgroundColor: '#f8f9fa',
          position: 'fixed',
          right: 0,
          top: '80px',
          zIndex: 100,
          overflowY: 'auto',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
          borderLeft: '1px solid #dee2e6'
        }}
      >
        <div style={{ padding: '20px' }}>
          <h3 style={{ margin: '0 0 15px 0', color: '#333' }}>Chat Sessions</h3>

          {loading && (
            <div style={{ textAlign: 'center', padding: '20px' }}>
              Loading sessions...
            </div>
          )}

          {error && (
            <div style={{
              color: 'red',
              padding: '10px',
              backgroundColor: '#ffe6e6',
              borderRadius: '4px',
              marginBottom: '10px'
            }}>
              {error}
            </div>
          )}

          {!loading && !error && sessions.length === 0 && (
            <div style={{ textAlign: 'center', padding: '20px', color: '#666' }}>
              No chat sessions yet
            </div>
          )}

          {!loading && !error && sessions.length > 0 && (
            <div style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
              {sessions.map((session) => (
                <div
                  key={session.id}
                  onClick={() => handleSessionClick(session)}
                  style={{
                    padding: '12px',
                    backgroundColor: selectedSession?.id === session.id ? '#007cba' : '#e9ecef',
                    color: selectedSession?.id === session.id ? 'white' : '#495057',
                    borderRadius: '5px',
                    cursor: 'pointer',
                    border: '1px solid #ced4da',
                    transition: 'background-color 0.2s'
                  }}
                >
                  <div style={{ fontWeight: 'bold', fontSize: '14px' }}>
                    {session.name}
                  </div>
                  <div style={{ fontSize: '12px', opacity: 0.8, marginTop: '4px' }}>
                    ID: {session.id.substring(0, 8)}...
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      </div>
    </>
  );
};

export default Sidenav;
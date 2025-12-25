// Base URL for the backend API
const API_BASE_URL = process.env.VITE_API_URL || 'http://localhost:8000';

// User signup
export const signup = async (userData: {
  email: string;
  password: string;
  level_of_experience: number;
}): Promise<{ success: boolean; error?: string }> => {
  try {
    const response = await fetch(`${API_BASE_URL}/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(userData),
      credentials: 'include', // Include cookies in the request
    });

    const data = await response.json();
    if (response.ok) {
      return { success: true };
    } else {
      return { success: false, error: data.message || 'Signup failed' };
    }
  } catch (error) {
    console.error('Signup error:', error);
    return { success: false, error: 'Network error occurred' };
  }
};

// User login
export const login = async (credentials: {
  email: string;
  password: string;
}): Promise<{ success: boolean; error?: string }> => {
  try {
    const response = await fetch(`${API_BASE_URL}/login`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(credentials),
      credentials: 'include', // Include cookies in the request
    });

    const data = await response.json();
    if (response.ok) {
      return { success: true };
    } else {
      return { success: false, error: data.message || 'Login failed' };
    }
  } catch (error) {
    console.error('Login error:', error);
    return { success: false, error: 'Network error occurred' };
  }
};

// Send a message to the chatbot
export const sendMessage = async (
  message: string,
  sessionId?: string
): Promise<{ success: boolean; response?: string; sessionId?: string; error?: string }> => {
  try {
    // Get selected text from the page
    const selectedText = window.getSelection()?.toString() || '';

    const response = await fetch(`${API_BASE_URL}/ask`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        session_id: sessionId || null,
        question: message,
        user_selected_text: selectedText,
      }),
      credentials: 'include', // Include cookies in the request
    });

    const data = await response.json();
    if (response.ok) {
      return {
        success: true,
        response: data.chat?.answer,
        sessionId: data.chat?.session_id
      };
    } else {
      return { success: false, error: data.message || 'Failed to get response' };
    }
  } catch (error) {
    console.error('Send message error:', error);
    return { success: false, error: 'Network error occurred' };
  }
};

// Get user's chat sessions
export const getSessions = async (): Promise<{ success: boolean; sessions?: any[]; error?: string }> => {

  try {
    const response = await fetch(`${API_BASE_URL}/get-sessions`, {
      method: 'GET',
      credentials: 'include', // Include cookies in the request
    });

    const data = await response.json();
    if (response.ok) {
      return { success: true, sessions: data.sessions };
    } 
    else if (response.status === 401) {
      return { success: false, error: 'User not authenticated' }
    } 
    else if (response.status === 404) {
      return { success: false, error: 'User not found' }
    } 
    else {
      return { success: false, error: data.message || 'Failed to get sessions' };
    }
  } catch (error) {
    console.error('Get sessions error:', error);
    return { success: false, error: 'Network error occurred' };
  }
};

// Get chats for a specific session
export const getChats = async (sessionId: string): Promise<{ success: boolean; chats?: any[]; error?: string }> => {
  try {
    const response = await fetch(`${API_BASE_URL}/get-chats`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ session_id: sessionId }),
      credentials: 'include', // Include cookies in the request
    });

    const data = await response.json();
    if (response.ok) {
      return { success: true, chats: data.chats };
    } else {
      return { success: false, error: data.message || 'Failed to get chats' };
    }
  } catch (error) {
    console.error('Get chats error:', error);
    return { success: false, error: 'Network error occurred' };
  }
};

// Helper function to logout user
export const logout = (): void => {
  // Clear the auth cookie
  document.cookie = 'access_token=; expires=Thu, 01 Jan 1970 00:00:00 UTC; path=/;';
};
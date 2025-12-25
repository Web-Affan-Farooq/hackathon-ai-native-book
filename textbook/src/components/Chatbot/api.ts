// This file is now deprecated since API calls are made directly from ChatWidget
// Keeping this file for reference or future use

const getAuthToken = (): string | null => {
 
  const cookies = document.cookie.split(';');
  for (let cookie of cookies) {
    const [name, value] = cookie.trim().split('=');
    if (name === 'access_token') {
      return value;
    }
  }
  return null;
};

// Helper function to check if user is authenticated
export const isAuthenticated = (): boolean => {
  return getAuthToken() !== null;
};
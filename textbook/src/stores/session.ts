import {create} from "zustand"
import { Session } from "../types"

interface ChatbotSessionState {
    isUserAuthenticated:boolean;
    setIsUserAuthenticated:(status: boolean) => void;
    sessions :Session[],
    setSession:(list:Session[]) => void
    selectedSession:null | Session,
    setSelectedSession:(session:Session) => void;
}

export const useSessions = create<ChatbotSessionState>()((set) => (
    {
        isUserAuthenticated:true,
        setIsUserAuthenticated:(status) => set({
            isUserAuthenticated:status
        }),
        sessions:[],
        setSession:(list) => set(() => ({
            sessions:list
        })),
        selectedSession:null,
        setSelectedSession:(session) => set(() => (
            {
                selectedSession:session
            }
        ))
    }
))
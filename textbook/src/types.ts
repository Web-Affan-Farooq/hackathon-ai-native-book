type User = {
    email:string,
    last_login:string,
    level_of_experience:number,
    chats:Record<string , Chat>
}

type Chat= {
    id:string,
    question:string,
    answer:string,
    created_at:string,
}

type Session = {
    name:string;
    id:string;
}

export type {
    Session,
}
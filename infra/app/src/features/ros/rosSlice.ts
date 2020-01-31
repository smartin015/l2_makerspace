import { createSlice, PayloadAction } from '@reduxjs/toolkit'

interface RosState {
  socket: WebSocket;
}

const rosSlice = createSlice({
  name: 'ros',
  initialState: {
    socket: new WebSocket("")
  } as RosState,
  reducers: {
    subscribeTopic(state, action: PayloadAction<{topic: string, cb: (payload: string)=>void}>) {
      console.log("todo subscribeTopic");
    },
    unsubscribeTopic(state, action: PayloadAction<string>) {
      console.log("todo unsubscribe");
    },
    publishTopic(state, action: PayloadAction<{topic: string, msg: Object}>) {
      console.log("todo publishTopic");
    },
  },
});

export const {subscribeTopic, unsubscribeTopic, publishTopic} = rosSlice.actions

import { h, render } from '/web_modules/preact.js'
import App from './components/App/index.js'
import { configureStore } from '/web_modules/@reduxjs/toolkit.js'
// import { Provider } from '/web_modules/preact-redux.js'
import {combineReducers} from '/web_modules/redux.js'
import Moment from '/web_modules/moment.js'
console.log(Moment().format());

const store = configureStore({
  reducer: combineReducers({}),
})

const appMount = document.querySelector('#app')
if (appMount) render(<App />, appMount)

export default App

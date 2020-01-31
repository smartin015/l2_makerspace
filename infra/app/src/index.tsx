declare namespace JSX {
    type Element = preact.JSX.Element;
    type HTMLAttributes = preact.JSX.HTMLAttributes;
}
import App from './components/App/index.js'
import { h, render } from '/web_modules/preact.js'
import { configureStore } from '@reduxjs/toolkit'
import { Provider } from 'preact-redux'
import rootReducer from './reducers'

const store = configureStore({
  reducer: rootReducer
})

const elem = document.getElementById('app')
if (elem !== null) {
  render(
    <Provider store={store}>
      <App />
    </Provider>,
    elem
  )
}
export default App

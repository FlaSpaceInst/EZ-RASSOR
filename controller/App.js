import React from 'react';
import ControllerStyle from './src/styles/controller.js';
import ControllerScreen from './src/components/app/ControllerScreen'

export default class App extends React.Component {

  render() {
    return(
      <ControllerScreen Style={ControllerStyle}/>
    )
  } 
} 
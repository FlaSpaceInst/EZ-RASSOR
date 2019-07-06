import React from 'react';
import {Animated} from 'react-native';

// Fade in amimation
export default class FadeInView extends React.Component { 
  state = { 
    fadeAnim: new Animated.Value(0),  
  }

  componentDidMount() {
    Animated.timing(
      this.state.fadeAnim,
      {
        toValue: 1,
        duration: 5000,
      }
    ).start();
  }

  render() {
    let { fadeAnim } = this.state;

    return (
      <Animated.View style={{ ...this.props.style, opacity: fadeAnim }}>
        {this.props.children}
      </Animated.View>
    );
  }
}
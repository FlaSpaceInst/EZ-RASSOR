import React from 'react';
import ControllerScreen from './src/components/app/ControllerScreen.js';
import RoboticArmScreen from './src/components/app/RoboticArmScreen.js';
import { NavigationContainer } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';

const Stack = createNativeStackNavigator();

export default class App extends React.Component {
  render() {
    return(
      <NavigationContainer>
      <Stack.Navigator
      screenOptions={{
        headerShown: false
      }}
       >
        <Stack.Screen name="Controller" component={ControllerScreen}/>
        <Stack.Screen name="Arm" component={RoboticArmScreen} />
      </Stack.Navigator>
     </NavigationContainer>
      // <ControllerScreen/>
      // <RoboticArmScreen/> */}
    );
  }
}
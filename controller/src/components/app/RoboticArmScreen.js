import React from 'react';
import Modal from "react-native-modal";
import FadeInView from "./FadeInView";
import EZRASSOR from '../../api/ezrassor-service' 
import ControllerStyle from '../../styles/controller';
import {Robot, Operation} from '../../enumerations/robot-commands';
import { StyleSheet, Linking, Text, View, TouchableHighlight, TouchableOpacity, Image, StatusBar, KeyboardAvoidingView, TextInput} from 'react-native';
import { FontAwesome, MaterialCommunityIcons } from '@expo/vector-icons';
import * as Font  from 'expo-font';

export default class RoboticArmScreen extends React.Component {


  constructor(props) {
    super(props);
    this.state = {
      autonomyModalVisible: false,
      infoModalVisible: false,
      ipModal: false,
      xyModal: false,
      armModal: false,
      isLoading: true,
      control: 0,
      xy: '0,0',
      ip: '192.168.0.102:8080' ,
      grabber_flag: 0
    }; 
    this.EZRASSOR = new EZRASSOR(this.state.ip);
  }

setAutonomyModalVisible(visible) {
  this.setState({ autonomyModalVisible: visible });
}
async componentDidMount () {
  await Font.loadAsync({
    'NASA': require('../../../assets/nasa.ttf'),
  });
  this.setState({ isLoading: false })
}
sendOperation(part, operation) {
  requestAnimationFrame(() => {
    this.EZRASSOR.executeRobotCommand(part, operation);
  });
}
    render() {
        return(      
          <View style={ControllerStyle.container}> 
        <StatusBar hidden />

        <Modal
          style={ControllerStyle.modalViewContainer}
          isVisible={this.state.autonomyModalVisible}
          onSwipeComplete={() => this.setAutonomyModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => this.setAutonomyModalVisible(!this.state.autonomyModalVisible)}>
    
          <TouchableHighlight style={{ flex: 1, marginHorizontal: 15, justifyContent: 'center' }}>
            <View>
              <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}>
                <Text style={ControllerStyle.textLarge}>Activate Autonomous Arm Function(s)</Text>
              </View>
              <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}> 
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.ARM, Operation.PICKUP)}}>
                    <Text style={ControllerStyle.textSmall}>Pick Up Paver</Text> 
                  </TouchableOpacity> 
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.ARM, Operation.PLACE)}}>
                    <Text style={ControllerStyle.textSmall}>Place Paver</Text>
                  </TouchableOpacity>
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.ARM, Operation.HOME)}}>
                    <Text style={ControllerStyle.textSmall}>Return Home</Text>
                  </TouchableOpacity>
              </View>
            </View>
          </TouchableHighlight>
        </Modal>
        
        <View style={ControllerStyle.headerContainer}>
        <TouchableOpacity style={{ flex: 1, padding: 3, }}onPress={() => { this.props.navigation.navigate('Controller')}} > 
            <MaterialCommunityIcons name="car-back" size={32} color='#fff'/>
          </TouchableOpacity>
          <Text style={ControllerStyle.textMedium}>EZ-RASSOR Robotic Arm Controller</Text>
          <TouchableOpacity style={{ flex: 1}}onPress={() => { this.setAutonomyModalVisible(true)} }> 
            <MaterialCommunityIcons style={{marginLeft: "auto"}} name="robot" size={32} color='#fff'/>
          </TouchableOpacity>
        </View>

        {/* Left D pad */}
        <View style={ControllerStyle.buttonLayoutContainer}>
        <View style={ControllerStyle.ArmContainer}> 
          <View style={{flex: 2 , flexDirection: 'column'}}>
            
            <View style={{flex: 2 , flexDirection: 'row'}}>
            <View style={ControllerStyle.wheelFunctionContainer}>
           <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.ARM, Operation.ARMUP)}} 
                 onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>  
               <FontAwesome name="chevron-up" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
             <View style={{flex: 2 , flexDirection: 'row'}}>
               <View style={ControllerStyle.dPadLeft}>
                 <TouchableOpacity
                     onPressIn={() => {this.sendOperation(Robot.ARM, Operation.ARMLEFT)}}
                     onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>
                   <FontAwesome name="chevron-left" size={50} color='#fff'/>
                 </TouchableOpacity>
               </View>
               <View style={ControllerStyle.dPadRight}>
                 <TouchableOpacity
                     onPressIn={() => {this.sendOperation(Robot.ARM, Operation.ARMRIGHT)}}
                     onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>
                   <FontAwesome name="chevron-right" size={50} color='#fff'/>
                 </TouchableOpacity>
               </View> 
               </View>
             <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.ARM, Operation.ARMDOWN)}} 
                 onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>  
               <FontAwesome name="chevron-down" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
             </View>
            <View style={ControllerStyle.wheelFunctionContainer}>
          {/* Right D pad */}
           <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.ARM, Operation.GRABBERUP)}} 
                 onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>  
               <FontAwesome name="chevron-up" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
             <View style={{flex: 2 , flexDirection: 'row'}}>
               <View style={ControllerStyle.dPadLeft}>
                 <TouchableOpacity
                     onPressIn={() => {this.sendOperation(Robot.ARM, Operation.GRABBERLEFT)}}
                     onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>
                   <FontAwesome name="chevron-left" size={50} color='#fff'/>
                 </TouchableOpacity>
               </View>
               <View style={ControllerStyle.dPadRight}>
                 <TouchableOpacity
                     onPressIn={() => {this.sendOperation(Robot.ARM, Operation.GRABBERRIGHT)}}
                     onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>
                   <FontAwesome name="chevron-right" size={50} color='#fff'/>
                 </TouchableOpacity>
               </View> 
               </View>
             <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.ARM, Operation.GRABBERDOWN)}} 
                 onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>  
               <FontAwesome name="chevron-down" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
             </View>
            <View style={ControllerStyle.wheelFunctionContainer}>
             <View style={{flex: 2 , flexDirection: 'column'}}>
             <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.ARM, Operation.ROTATELEFT)}} 
                 onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>  
               <FontAwesome name="rotate-right" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
               <View style={ControllerStyle.dPadLeft}>
                 <TouchableOpacity
                     onPressIn={() => {
                      if (this.grabber_flag == 0) {
                        this.sendOperation(Robot.ARM, Operation.close);
                        this.grabber_flag = 1;
                      } else {
                        this.sendOperation(Robot.ARM, Operation.open);
                        this.grabber_flag = 0;
                      }
                      }}>
                   <Text style={ControllerStyle.textSmallCenter}>Open/ Close Grabber</Text>
                 </TouchableOpacity>
               </View>
               <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.ARM, Operation.ROTATELEFT)}} 
                 onPressOut={() => {this.sendOperation(Robot.ARM, Operation.STOPARM)}}>  
               <FontAwesome name="rotate-left" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
               </View>
             </View>
             </View>
           </View>
           </View>
        </View>
      </View>

        );
    }
}
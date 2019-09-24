import React from 'react';
import Modal from "react-native-modal";
import FadeInView from "ezrassor-app/src/components/app/FadeInView";
import EZRASSOR from 'ezrassor-app/src/api/ezrassor-service' 
import ControllerStyle from 'ezrassor-app/src/styles/controller';
import {Robot, Operation} from 'ezrassor-app/src/enumerations/robot-commands';
import { Linking, Text, View, TouchableHighlight, TouchableOpacity, Image, StatusBar, KeyboardAvoidingView, TextInput} from 'react-native';
import { FontAwesome, MaterialCommunityIcons } from '@expo/vector-icons';
import * as Font  from 'expo-font';

export default class ControllerScreen extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      autonomyModalVisible: false,
      infoModalVisible: false,
      ipModal: false,
      xyModal: false,
      isLoading: true,
      control: 0,
      xy: '0,0',
      ip: '192.168.1.2:8080' 
    }; 

    this.EZRASSOR = new EZRASSOR(this.state.ip);
  }

  async componentDidMount () {
    await Font.loadAsync({
      'NASA': require('../../../assets/nasa.ttf'),
    });
    this.setState({ isLoading: false })
  }

  setAutonomyModalVisible(visible) {
    this.setState({ autonomyModalVisible: visible });
  }

  setInfoModalVisible(visible) {
    this.setState({ infoModalVisible: visible });
  }

  setIPModalVisible(visible){
    this.setState({ipModal: visible});
  }

  setXYModalVisible(visible){
    this.setState({xyModal:visible});
  }

  changeXY(combined){
    this.setState({xy:combined}, () => {
      this.EZRASSOR.setCoordinate(this.state.xy);
    });
  }

  changeIP(text){
    this.setState({ip:text}, () => {
      this.EZRASSOR.host = this.state.ip;
    });
  }

  // Update animation frame before processing click so that opacity can change on click 
  sendOperation(part, operation) {
    requestAnimationFrame(() => {
      this.EZRASSOR.executeRobotCommand(part, operation);
    });
  }

  render() {

    // Loading font
    if (this.state.isLoading) {
      return (
        <View style={{flex: 1, backgroundColor: '#5D6061'}}/>
      );
    }

    return ( 
      <View style={ControllerStyle.container}> 
        <StatusBar hidden />

        {/* Autonomy Popup Modal*/}
        <Modal
          style={ControllerStyle.modalViewContainer}
          isVisible={this.state.autonomyModalVisible}
          onRequestClose={() => this.setAutonomyModalVisible(!this.state.autonomyModalVisible)}>
          
          <TouchableHighlight style={{ flex: 1, marginHorizontal: 15, justifyContent: 'center' }}>
            <View>
              <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}>
                <Text style={ControllerStyle.textLarge}>Activate Autonomout Function(s)</Text>
              </View>
              <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}> 
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={()=>this.setXYModalVisible(true)}>
                    <Text style={ControllerStyle.textSmall}>Drive</Text>
                  </TouchableOpacity>
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.AUTONOMY, Operation.DIG)}}>
                    <Text style={ControllerStyle.textSmall}>Dig</Text> 
                  </TouchableOpacity> 
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.AUTONOMY, Operation.DUMP)}}>
                    <Text style={ControllerStyle.textSmall}>Dump</Text>
                  </TouchableOpacity>
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.AUTONOMY, Operation.SELFRIGHT)}}>
                    <Text style={[ControllerStyle.textSmall, ControllerStyle.columnText]}>Self -    Right</Text>
                  </TouchableOpacity>
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.AUTONOMY, Operation.FULLAUTONOMY)}}>
                    <Text style={[ControllerStyle.textSmall, ControllerStyle.columnText]}>Auto      Mode</Text>
                  </TouchableOpacity>
              </View>
            </View>
          </TouchableHighlight>
        </Modal>

        {/*Information Popup Modal*/}
        <Modal
          style={ControllerStyle.modalViewContainer}
          isVisible={this.state.infoModalVisible}
          onSwipe={() => this.setInfoModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => this.setInfoModalVisible(false)}>
          <View style={{flexDirection: 'row'}}>
            <View style={ControllerStyle.columnHeader}>
              <Text style={ControllerStyle.textSmall}>App Developer</Text>
              <Text style={{color: '#fff'}}>Christopher Taliaferro</Text>
              <View style={{marginVertical: 10}}/>
              <Text style={ControllerStyle.textSmall}>EZ-RASSOR Team</Text>
              <View style={{flexDirection: 'row'}}>
                <View>
                  <Text style={ControllerStyle.columnText}>Camilo Lozano</Text>
                  <Text style={ControllerStyle.columnText}>Cameron Taylor</Text>
                  <Text style={ControllerStyle.columnText}>Harrison Black</Text>
                  <Text style={ControllerStyle.columnText}>Ron Marrero</Text>
                  <Text style={ControllerStyle.columnText}>Samuel Lewis</Text>
                </View>
                <View style={{marginHorizontal:5}}/>
                <View>
                  <Text style={ControllerStyle.columnText}>Sean Rapp</Text>
                  <Text style={ControllerStyle.columnText}>Tiger Sachse</Text>
                  <Text style={ControllerStyle.columnText}>Tyler Duncan</Text>
                  <Text style={ControllerStyle.columnText}>Lucas Gonzalez</Text>
                </View>
              </View>
            </View>
            <View style={{ flex: .5, borderRadius:20, backgroundColor: '#2e3030'}}></View>
            <View style={ControllerStyle.columnHeader}>
              <Text style={ControllerStyle.textSmall}>Our Mission</Text>
              <View style={{marginVertical: 10}}/>
              <Text style={ControllerStyle.columnText}>The EZ-RASSOR (EZ Regolith Advanced Surface Systems Operations Robot) is an inexpensive, autonomous, regolith-mining robot designed to mimic the look and abilities of NASA’s RASSOR on a smaller scale. The primary goal of the EZ-RASSOR is to provide a functioning demonstration robot for visitors at the Kennedy Space Center.</Text>
              <View style={{marginVertical: 10}}/>
              <Text style={[ControllerStyle.textTiny, ControllerStyle.columnText]}>Visit
                  <Text style={[ControllerStyle.textTiny, ControllerStyle.columnHyperlink]} onPress={() => Linking.openURL('https://github.com/FlaSpaceInst/EZ-RASSOR')}> our GitHub repository </Text>
                  for more information
              </Text>
            </View>
          </View>
        </Modal>

        {/*Settings Modal*/}
        <Modal
          style={ControllerStyle.modalViewContainer}
          isVisible={this.state.ipModal}
          onSwipe={() => this.setIPModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => {this.setIPModalVisible(false)}}>
          <KeyboardAvoidingView
            paddingLeft={34}
            paddingRight={34}>
            <Text style={[ControllerStyle.textLarge, ControllerStyle.columnText]}>EZ-RASSOR Host to Control</Text>
            <TextInput
              style={ControllerStyle.ipInputBox}
              onChangeText={(text) => this.changeIP(text)}
              value={this.state.ip}
              marginVertical={20} />
          </KeyboardAvoidingView>
        </Modal>
        <Modal
          style={ControllerStyle.modalViewContainer}
          isVisible={this.state.xyModal}
          onSwipe={() => this.setXYModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => {this.setXYModalVisible(false)}}>
          <KeyboardAvoidingView
            paddingLeft={64}
            paddingRight={64}>
            <Text style={[ControllerStyle.textSmall, ControllerStyle.columnText]}>Enter the X,Y coordinates where the robot will drive to</Text>
            <TextInput style={ControllerStyle.ipInputBox} onChangeText={(text) => this.changeXY(text)} value={this.state.xy} placeholder='x,y' marginVertical={20} />
              <TouchableOpacity style={{alignItems: 'center', backgroundColor: '#DDDDDD', padding: 10}}
                  onPress={()=> {
                      this.sendOperation(Robot.AUTONOMY, Operation.DRIVE);
                      this.setXYModalVisible(false);}}>
                    <Text>Done</Text>
              </TouchableOpacity>
          </KeyboardAvoidingView>
        </Modal>

        {/* Controller screen top row controls */}
        <FadeInView style={ControllerStyle.headerContainer}>

          {/*Left Row Icons*/}
          <TouchableOpacity style={{ flex: 1, padding: 3 }} onPress={() => { this.setInfoModalVisible(true)}}>
            <FontAwesome name="info-circle" size={32} color='#fff'/>
          </TouchableOpacity>
          <TouchableOpacity style={{ flex: 1, padding: 1 }} onPress={() => this.setIPModalVisible(true)}>
            <FontAwesome name="search" size={30} color='#fff'/>
          </TouchableOpacity>

          <Text style={ControllerStyle.textMedium}>EZ-RASSOR Controller</Text>

          {/*Right Row Icons*/}
          <TouchableOpacity style={{ flex: 1, padding: 3}} onPress={() => {this.sendOperation(Robot.ALL, Operation.STOP)}}>
            <FontAwesome style={{marginLeft: "auto"}} name="stop-circle-o" size={35} color='#fff'/>
          </TouchableOpacity>
          <TouchableOpacity style={{ flex: 1, padding: 3, }}onPress={() => { this.setAutonomyModalVisible(true)}} > 
            <MaterialCommunityIcons style={{marginLeft: "auto"}} name="robot" size={32} color='#fff'/>
          </TouchableOpacity>
          
        </FadeInView>

        {/*Wheel Operations Board*/}
        <FadeInView style={ControllerStyle.buttonLayoutContainer}>
          <View style={ControllerStyle.wheelFunctionContainer}>
            <View style={ControllerStyle.upAndDownDPad} >
            <TouchableOpacity 
                onPressIn={() => {this.sendOperation(Robot.WHEELS, Operation.DRIVEFORWARD)}} 
                onPressOut={() => {this.sendOperation(Robot.WHEELS, Operation.STOPWHEELS)}}>  
              <FontAwesome name="chevron-up" size={50} color='#fff'/>
            </TouchableOpacity>
            </View>
            <View style={{flex: 2 , flexDirection: 'row'}}>
              <View style={ControllerStyle.dPadLeft}>
                <TouchableOpacity
                    onPressIn={() => {this.sendOperation(Robot.WHEELS, Operation.TURNLEFT)}}
                    onPressOut={() => {this.sendOperation(Robot.WHEELS, Operation.STOPWHEELS)}}>
                  <FontAwesome name="chevron-left" size={50} color='#fff'/>
                </TouchableOpacity>
              </View>
              <View style={ControllerStyle.dPadRight}>
                <TouchableOpacity
                    onPressIn={() => {this.sendOperation(Robot.WHEELS, Operation.TURNRIGHT)}}
                    onPressOut={() => {this.sendOperation(Robot.WHEELS, Operation.STOPWHEELS)}}>
                  <FontAwesome name="chevron-right" size={50} color='#fff'/>
                </TouchableOpacity>
              </View>
            </View>
            <View style={ControllerStyle.upAndDownDPad}>
              <TouchableOpacity
                  onPressIn={() => {this.sendOperation(Robot.WHEELS, Operation.DRIVEBACKWARD)}}
                  onPressOut={() => {this.sendOperation(Robot.WHEELS, Operation.STOPWHEELS)}}>
                <FontAwesome name="chevron-down" size={50} color='#fff'/>
              </TouchableOpacity>
            </View>
          </View>

          {/*Drum/Arm Operations Board*/}
          <View style={ControllerStyle.drumFunctionContainer}> 
            <View style= {{ flex: 8}}>
              <View style={{ flexDirection: 'row' }}>
                <View style={{ flexDirection: 'row' }}>
                  <View>
                    <TouchableOpacity
                        onPressIn={() => {this.sendOperation(Robot.FRONTARM, Operation.UP)}}
                        onPressOut={() => {this.sendOperation(Robot.FRONTARM, Operation.STOP)}}>
                      <FontAwesome name="arrow-circle-up" size={50} color='#fff'/>
                    </TouchableOpacity>
                  </View>
                  <View style={{ marginHorizontal: 15 }}> 
                    <TouchableOpacity
                        onPressIn={() => {this.sendOperation(Robot.FRONTARM, Operation.DOWN)}}
                        onPressOut={() => {this.sendOperation(Robot.FRONTARM, Operation.STOP)}}>
                      <FontAwesome name="arrow-circle-down" size={50} color='#fff'/>
                    </TouchableOpacity>
                  </View>
                </View>
                <View style={ControllerStyle.rightSideRow}>
                  <View style={{ marginHorizontal: 15 }}>
                    <TouchableOpacity 
                        onPressIn={() => {this.sendOperation(Robot.BACKARM, Operation.UP)}}
                        onPressOut={() => {this.sendOperation(Robot.BACKARM, Operation.STOP)}}>
                      <FontAwesome name="arrow-circle-up" size={50} color='#fff'/>
                    </TouchableOpacity>
                  </View>
                  <View>
                    <TouchableOpacity 
                        onPressIn={() => {this.sendOperation(Robot.BACKARM, Operation.DOWN)}}
                        onPressOut={() => {this.sendOperation(Robot.BACKARM, Operation.STOP)}}>
                      <FontAwesome name="arrow-circle-down" size={50} color='#fff'/>
                    </TouchableOpacity>
                  </View>
                </View>
              </View>
              <Image style={ControllerStyle.image} source={require('../../../assets/rassor.png')}/>
              <View style={{ flexDirection: 'row' }}>
                <View style={{ flexDirection: 'row' }}>
                  <View>
                    <TouchableOpacity
                      onPressIn={() => {this.sendOperation(Robot.FRONTDRUM, Operation.ROTATEOUTWARD)}}
                      onPressOut={() => {this.sendOperation(Robot.FRONTDRUM, Operation.STOP)}}>
                      <FontAwesome name="rotate-left" size={50} color='#fff'/>
                    </TouchableOpacity>
                  </View>
                  <View style={{ marginHorizontal: 15 }}>
                    <TouchableOpacity
                      onPressIn={() => {this.sendOperation(Robot.FRONTDRUM, Operation.ROTATEINWARD)}}
                      onPressOut={() => {this.sendOperation(Robot.FRONTDRUM, Operation.STOP)}}>
                      <FontAwesome name="rotate-right" size={50} color='#fff'/>
                    </TouchableOpacity>
                  </View>
                </View>
                <View style={ControllerStyle.rightSideRow}>
                  <View style={{ marginHorizontal: 15 }}>
                    <TouchableOpacity
                      onPressIn={() => {this.sendOperation(Robot.BACKDRUM, Operation.ROTATEINWARD)}}
                      onPressOut={() => {this.sendOperation(Robot.BACKDRUM, Operation.STOP)}}>
                      <FontAwesome name="rotate-left" size={50} color='#fff'/>
                    </TouchableOpacity>
                  </View>
                  <View>
                    <TouchableOpacity
                      onPressIn={() => {this.sendOperation(Robot.BACKDRUM, Operation.ROTATEOUTWARD)}}
                      onPressOut={() => {this.sendOperation(Robot.BACKDRUM, Operation.STOP)}}>
                      <FontAwesome name="rotate-right" size={50} color='#fff'/>
                    </TouchableOpacity>
                  </View>
                </View>
              </View>
            </View>
          </View>
        </FadeInView>
      </View>
    );
  }
} 

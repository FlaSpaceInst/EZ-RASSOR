import React from 'react';
import Modal from "react-native-modal";
import FadeInView from "./FadeInView";
import { Animated, StyleSheet, Text, View, TouchableHighlight, TouchableOpacity, Image, Button, StatusBar, KeyboardAvoidingView, TextInput} from 'react-native';
import { FontAwesome, MaterialCommunityIcons } from '@expo/vector-icons';
import { Font } from 'expo';

export default class ControllerScreen extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      modalVisible: false,
      modal2Visible: false,
      ipModal: false,
      xyModal: false,
      isLoading: true,
      ip:'128.217.243.159:8080',  
      endpoint: '/',
      control: 0,
      xy: '(0,0)', 
      twist: {
        autonomous_toggles:0,
        target_coordinate:{
              x:0,y:0
        },
        wheel_instruction: "none",
        front_arm_instruction:0,
        back_arm_instruction:0,
        front_drum_instruction:0,
        back_drum_instruction:0
       },


    }; 
    this.handleSubmit = this.handleSubmit.bind(this);

    this.style = this.props.Style;
  }

  async componentDidMount () {
    await Font.loadAsync({
      'NASA': require('../../../assets/nasa.ttf'),
    });
    this.setState({ isLoading: false })
  }

  setModalVisible(visible) {
    this.setState({ modalVisible: visible });
  }

  setModal2Visible(visible) {
    this.setState({ modal2Visible: visible });
  }

  setIPModalVisible(visible){
    this.setState({ipModal: visible});
  }

  setXYModalVisible(visible){
    this.setState({xyModal:visible});
  }

  changeXY(text){
    this.setState({xy:text})
  }
  changeIP(text){
    this.setState({ip:text})
  }

  controlUpdate(input){
    
    newControl = this.state.control + input
    
    this.setState({control: newControl}, ()=> {
      console.log(this.state.control)
      this.handleSubmit(this.state.control)
    })
  
  }

   twistUpdate(input){
       this.setState({twist:input},()=>{
           msg=this.state.twist
           msg = JSON.stringify(msg)
           console.log(msg)
           this.handleSubmit(msg)
       }) 

   }

   sendXY(){
      xy = this.state.xy
      console.log(xy)
   }
  
  handleSubmit(event){

    url = 'http://'+this.state.ip+this.state.endpoint
    console.log(url)
    
    return fetch(
      url,
      {
        headers: {"Content-Type":"text/plain; charset=utf-8"},
        method: 'POST',
        headers:{
          Accept: 'application/json',
        },
        body: event.toString()
      }
    )
    .catch((error) => {
      //alert("Unable to connect to EZ-RASSOR");
      console.log(error);
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
      <View style={this.style.container}>
        <StatusBar hidden />
        <Modal
          style={this.style.modalViewContainer}
          isVisible={this.state.modalVisible}
          onSwipe={() => this.setModalVisible(!this.state.modalVisible)}
          swipeDirection='down'
          onRequestClose={() => this.setModalVisible(!this.state.modalVisible)}
          >
          <TouchableHighlight style={{ flex: 1, marginHorizontal: 15, justifyContent: 'center' }}>
            <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}>
                <TouchableOpacity style={this.style.modalButton} onPress={()=>this.setXYModalVisible(true)}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Drive</Text>
              </TouchableOpacity>
              <TouchableOpacity style={this.style.modalButton} onPress={()=>this.twistUpdate({autonomous_toggles:0b10})}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Dig</Text>
              </TouchableOpacity>
              <TouchableOpacity style={this.style.modalButton} onPress={()=>this.twistUpdate({autonomous_toggles:0b100})}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Dump</Text>
              </TouchableOpacity>
              <TouchableOpacity style={this.style.modalButton} onPress={()=>this.twistUpdate({autonomous_toggles:0b1000})}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Self-Right</Text>
              </TouchableOpacity>
              <TouchableOpacity style={this.style.modalButton} onPress={()=>this.twistUpdate({autonomous_toggles:0b10000})}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Full-Autonomy</Text>
              </TouchableOpacity>
            </View>
          </TouchableHighlight>
        </Modal>

        <Modal
          style={this.style.modalViewContainer}
          isVisible={this.state.modal2Visible}
          onSwipe={() => this.setModal2Visible(false)}
          swipeDirection='down'
          onRequestClose={() => this.setModal2Visible(false)}
          >
          <View style={{flexDirection: 'row'}}>
            <View style={{ flex: 20, marginHorizontal: 15, justifyContent: 'center', alignItems: 'center' }}>
              <Text style={{fontSize:20, fontWeight: 'bold', color: '#fff', textAlign: 'center'}}>NASA EZ-RASSOR Controller</Text>
              <View style={{marginVertical: 10}}/>
              <Text style={{fontSize:15, fontWeight: 'bold', color: '#fff'}}>App Developer</Text>
              <Text style={{color: '#fff'}}>Christopher Taliaferro</Text>
              <View style={{marginVertical: 10}}/>
              <Text style={{fontSize:15, fontWeight: 'bold', color: '#fff'}}>EZ-RASSOR Team</Text>
              <View style={{flexDirection: 'row'}}>
                <View>
                  <Text style={{color: '#fff', textAlign: 'center'}}>Camilo Lozano</Text>
                  <Text style={{color: '#fff', textAlign: 'center'}}>Cameron Taylor</Text>
                  <Text style={{color: '#fff', textAlign: 'center'}}>Harrison Black</Text>
                  <Text style={{color: '#fff', textAlign: 'center'}}>Ron Marrero</Text>
                  <Text style={{color: '#fff', textAlign: 'center'}}>Samuel Lewis</Text>
                </View>
                <View style={{marginHorizontal:5}}/>
                <View>
                  <Text style={{color: '#fff', textAlign: 'center'}}>Sean Rapp</Text>
                  <Text style={{color: '#fff', textAlign: 'center'}}>Tiger Sachse</Text>
                  <Text style={{color: '#fff', textAlign: 'center'}}>Tyler Duncan</Text>
                  <Text style={{color: '#fff', textAlign: 'center'}}>Lucas Gonzalez</Text>
                </View>
              </View>
            </View>
            <View style={{ flex: .5, borderRadius:20, backgroundColor: '#2e3030'}}></View>
            <View style={{ flex: 20, marginHorizontal: 15, justifyContent: 'center', alignItems: 'center' }}>
              <Text style={{fontSize:20, fontWeight: 'bold', color: '#fff'}}>Our Mission</Text>
              <View style={{marginVertical: 10}}/>
              <Text style={{color: '#fff', textAlign: 'center'}}>The EZ-RASSOR (EZ Regolith Advanced Surface Systems Operations Robot) is an inexpensive, autonomous, regolith-mining robot designed to mimic the look and abilities of NASA’s RASSOR on a smaller scale. The primary goal of the EZ-RASSOR is to provide a functioning demonstration robot for visitors at the Kennedy Space Center.</Text>
            </View>
          </View>
        </Modal>

        <Modal
          style={this.style.modalViewContainer}
          isVisible={this.state.ipModal}
          onSwipe={() => this.setIPModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => {this.setIPModalVisible(false)}}>
          <KeyboardAvoidingView
            paddingLeft={64}
            paddingRight={64}>
            <Text style={{color: '#fff', textAlign: 'center', fontFamily: 'NASA', fontSize: 45,}}>IP ADDRESS</Text>
            <TextInput
              style={this.style.ipInputBox}
              onChangeText={(text) => this.changeIP(text)}
              value={this.state.ip}
              marginVertical={20} />
          </KeyboardAvoidingView>
        </Modal>
        <Modal
          style={this.style.modalViewContainer}
          isVisible={this.state.xyModal}
          onSwipe={() => this.setXYModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => {this.setXYModalVisible(false)}}>
          <KeyboardAvoidingView
            paddingLeft={64}
            paddingRight={64}>
            <Text style={{color: '#fff', textAlign: 'center', fontFamily: 'NASA', fontSize: 45,}}>(X,Y)</Text>
            <TextInput
              style={this.style.ipInputBox}
              onChangeText={(text) => this.changeXY(text)}
              value={this.state.xy}
              marginVertical={20} />
              <TouchableOpacity style={{alignItems: 'center', backgroundColor: '#DDDDDD', padding: 10}}
                  onPress={()=> this.sendXY()}>
                    <Text>Done</Text>
              </TouchableOpacity>
          </KeyboardAvoidingView>
        </Modal>

        <FadeInView style={this.style.headerContainer}>
          <TouchableOpacity style={{ flex: 1, padding: 3 }}>
            <FontAwesome
              name="info-circle"
              size={32}
              color='#fff'
              onPress={() => { this.setModal2Visible(true); }}
            />
          </TouchableOpacity>
          <TouchableOpacity style={{ flex: 1, padding: 3 }}>
            <FontAwesome
              name="search"
              size={30}
              color='#fff'
              onPress={() => this.setIPModalVisible(true)}
            />
          </TouchableOpacity>
          <Text style={this.style.text}>EZ-RASSOR Controller</Text>
          <TouchableOpacity style={{ flex: 1, padding: 3}}>
            <MaterialCommunityIcons
              style={{marginLeft: "auto"}}
              name="close-octagon"
              size={35}
              color='#fff'
              onPress={() => {
                this.setState({control: 0}, ()=> this.handleSubmit(0))
              }}
            />
          </TouchableOpacity>
          <TouchableOpacity style={{ flex: 1, padding: 3, }}>
            <MaterialCommunityIcons
              style={{marginLeft: "auto"}}
              name="robot"
              size={32}
              color='#fff'
              onPress={() => { this.setModalVisible(true); }}
            />
          </TouchableOpacity>
        </FadeInView>

        <FadeInView style={this.style.buttonLayoutContainer}>
          <View style={{ flex: 3,  marginLeft: 10, borderRadius: 10, elevation: 3, backgroundColor: '#2e3030' }}>
            <View style={this.style.upAndDownDPad} 
            onTouchStart={() => this.twistUpdate({wheel_instruction:"forward"}) }
            onTouchEnd={() => this.twistUpdate({wheel_instruction:"stop"}) }
            >
            <TouchableOpacity>  
              <FontAwesome
                name="chevron-up"
                size={50}
                color='#fff'
              />
            </TouchableOpacity>
            </View>
            <View style={{flex: 2 , flexDirection: 'row'}}>
              <View style={this.style.dPadLeft}
              onTouchStart={() => this.twistUpdate({wheel_instruction:"left"}) }
              onTouchEnd={() => this.twistUpdate({wheel_instruction:"stop"}) }
              >
                <TouchableOpacity>
                  <FontAwesome
                    name="chevron-left"
                    size={50}
                    color='#fff'
                  />
                </TouchableOpacity>
              </View>
              <View style={this.style.dPadRight} 
              onTouchStart={() => this.twistUpdate({wheel_instruction:"right"}) }
              onTouchEnd={() => this.twistUpdate({wheel_instruction:"stop"}) }
              >
                <TouchableOpacity>
                  <FontAwesome
                    name="chevron-right"
                    size={50}
                    color='#fff'
                  />
                </TouchableOpacity>
              </View>
            </View>
            <View style={this.style.upAndDownDPad}
            onTouchStart={() => this.twistUpdate({wheel_instruction:"backward"}) }
            onTouchEnd={() => this.twistUpdate({wheel_instruction:"stop"}) }
            >
              <TouchableOpacity>
                <FontAwesome
                  name="chevron-down"
                  size={50}
                  color='#fff'
                />
              </TouchableOpacity>
            </View>
          </View>

          <View style={this.style.drumFunctionContainer}> 
            <View style= {{ flex: 8}}>
              <View style={{ flexDirection: 'row' }}>
                <View style={{ flexDirection: 'row' }}>
                  <View 
                  onTouchStart={() => this.twistUpdate({front_arm_instruction:1}) }
                  onTouchEnd={() => this.twistUpdate({front_arm_instruction:0}) }
                  >
                    <TouchableOpacity>
                      <FontAwesome
                        name="arrow-circle-up"
                        size={50}
                        color='#fff'
                      />
                    </TouchableOpacity>
                  </View>
                  <View style={{ marginHorizontal: 15 }} 
                  onTouchStart={() => this.twistUpdate({front_arm_instruction:-1}) }
                  onTouchEnd={() => this.twistUpdate({front_arm_instruction:0}) }
                  >
                    <TouchableOpacity>
                      <FontAwesome
                        name="arrow-circle-down"
                        size={50}
                        color='#fff'
                      />
                    </TouchableOpacity>
                  </View>
                </View>
                <View style={{ flexDirection: 'row', position: 'absolute', right: 0 }}>
                  <View style={{ marginHorizontal: 15 }} 
                  onTouchStart={() => this.twistUpdate({back_arm_instruction:1}) }
                  onTouchEnd={() => this.twistUpdate({back_arm_instruction:0}) }
                  >
                    <TouchableOpacity>
                      <FontAwesome
                        name="arrow-circle-up"
                        size={50}
                        color='#fff'
                      />
                    </TouchableOpacity>
                  </View>
                  <View 
                  onTouchStart={() => this.twistUpdate({back_arm_instruction:-1}) }
                  onTouchEnd={() => this.twistUpdate({back_arm_instruction:0}) }
                  >
                    <TouchableOpacity>
                      <FontAwesome
                        name="arrow-circle-down"
                        size={50}
                        color='#fff'
                      />
                    </TouchableOpacity>
                  </View>
                </View>
              </View>
              <Image style={this.style.image} source={require('../../../assets/rassor.png')}/>
              <View style={{ flexDirection: 'row' }}>
                <View style={{ flexDirection: 'row' }}>
                  <View 
                  onTouchStart={() => this.twistUpdate({front_drum_instruction:-1}) }
                  onTouchEnd={() => this.twistUpdate({front_drum_instruction:0}) }
                  >
                    <TouchableOpacity>
                      <FontAwesome
                        name="rotate-left"
                        size={50}
                        color='#fff'
                      />
                    </TouchableOpacity>
                  </View>
                  <View style={{ marginHorizontal: 15 }} 
                  onTouchStart={() => this.twistUpdate({front_drum_instruction:1}) }
                  onTouchEnd={() => this.twistUpdate({front_drum_instruction:0}) }
                  >
                    <TouchableOpacity>
                      <FontAwesome
                        name="rotate-right"
                        size={50}
                        color='#fff'
                      />
                    </TouchableOpacity>
                  </View>
                </View>
                <View style={{ flexDirection: 'row', position: 'absolute', right: 0 }}>
                  <View style={{ marginHorizontal: 15 }} 
                  onTouchStart={() => this.twistUpdate({back_drum_instruction:-1}) }
                  onTouchEnd={() => this.twistUpdate({back_drum_instruction:0}) }
                  >
                    <TouchableOpacity>
                      <FontAwesome
                        name="rotate-left"
                        size={50}
                        color='#fff'
                      />
                    </TouchableOpacity>
                  </View>
                  <View 
                  onTouchStart={() => this.twistUpdate({back_drum_instruction:1}) }
                  onTouchEnd={() => this.twistUpdate({back_drum_instruction:0}) }
                  >
                    <TouchableOpacity>
                      <FontAwesome
                        name="rotate-right"
                        size={50}
                        color='#fff'
                      />
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
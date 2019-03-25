import React from 'react';
import { Animated, StyleSheet, Text, View, TouchableHighlight, TouchableOpacity, Image, Button, StatusBar, KeyboardAvoidingView, TextInput} from 'react-native';
import { FontAwesome, MaterialCommunityIcons } from '@expo/vector-icons';
import Modal from "react-native-modal";
import { Font } from 'expo';

// Fade in amimation
class FadeInView extends React.Component { 
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

export default class App extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      modalVisible: false,
      modal2Visible: false,
      ipModal: false,
      isLoading: true,
      ip:'192.168.60.158',  
      endpoint: '/manual',
      control: 0,
    };
    this.handleSubmit = this.handleSubmit.bind(this);
  }

  async componentDidMount () {
    await Font.loadAsync({
      'NASA': require('./assets/nasa.ttf'),
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
  
  handleSubmit(event){

    url = 'http://'+this.state.ip+':5000'+this.state.endpoint
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
      <View style={styles.container}>
        <StatusBar hidden />
        <Modal
          style={styles.modalViewContainer}
          isVisible={this.state.modalVisible}
          onSwipe={() => this.setModalVisible(!this.state.modalVisible)}
          swipeDirection='down'
          onRequestClose={() => this.setModalVisible(!this.state.modalVisible)}
          >
          <TouchableHighlight style={{ flex: 1, marginHorizontal: 15, justifyContent: 'center' }}>
            <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}>
              <TouchableOpacity style={styles.modalButton}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Auto Dig</Text>
              </TouchableOpacity>
              <TouchableOpacity style={styles.modalButton}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Auto Dump</Text>
              </TouchableOpacity>
              <TouchableOpacity style={styles.modalButton}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Self Right</Text>
              </TouchableOpacity>
              <TouchableOpacity style={styles.modalButton}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Z Config</Text>
              </TouchableOpacity>
              <TouchableOpacity style={styles.modalButton}>
                <Text adjustsFontSizeToFit numberOfLines={1} style={{ fontWeight: 'bold', color: '#fff' }}>Explore</Text>
              </TouchableOpacity>
            </View>
          </TouchableHighlight>
        </Modal>

        <Modal
          style={styles.modalViewContainer}
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
          style={styles.modalViewContainer}
          isVisible={this.state.ipModal}
          onSwipe={() => this.setIPModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => {this.setIPModalVisible(false)}}>
          <KeyboardAvoidingView
            paddingLeft={64}
            paddingRight={64}>
            <Text style={{color: '#fff', textAlign: 'center', fontFamily: 'NASA', fontSize: 45,}}>IP ADDRESS</Text>
            <TextInput
              style={styles.ipInputBox}
              onChangeText={(text) => this.changeIP(text)}
              value={this.state.ip}
              marginVertical={20} />
          </KeyboardAvoidingView>
        </Modal>

        <FadeInView style={styles.headerContainer}>
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
          <Text style={styles.text}>EZ-RASSOR Controller</Text>
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

        <FadeInView style={styles.buttonLayoutContainer}>
          <View style={{ flex: 3,  marginLeft: 10, borderRadius: 10, elevation: 3, backgroundColor: '#2e3030' }}>
            <View style={styles.upAndDownDPad} 
            onTouchStart={() => this.controlUpdate(1<<11) }
            onTouchEnd={() => this.controlUpdate(-1<<11)}
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
              <View style={styles.dPadLeft} 
              onTouchStart={() => this.controlUpdate(1<<9) }
              onTouchEnd={() => this.controlUpdate(-1<<9)}
              >
                <TouchableOpacity>
                  <FontAwesome
                    name="chevron-left"
                    size={50}
                    color='#fff'
                  />
                </TouchableOpacity>
              </View>
              <View style={styles.dPadRight} 
              onTouchStart={() => this.controlUpdate(1<<8) }
              onTouchEnd={() => this.controlUpdate(-1<<8)}
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
            <View style={styles.upAndDownDPad}
            onTouchStart={() => this.controlUpdate(1<<10) }
            onTouchEnd={() => this.controlUpdate(-1<<10)}
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

          <View style={styles.drumFunctionContainer}> 
            <View style= {{ flex: 8}}>
              <View style={{ flexDirection: 'row' }}>
                <View style={{ flexDirection: 'row' }}>
                  <View 
                  onTouchStart={() => this.controlUpdate(1<<7) }
                  onTouchEnd={() => this.controlUpdate(-1<<7)}
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
                  onTouchStart={() => this.controlUpdate(1<<6) }
                  onTouchEnd={() => this.controlUpdate(-1<<6)}
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
                  onTouchStart={() => this.controlUpdate(1<<5) }
                  onTouchEnd={() => this.controlUpdate(-1<<5)}
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
                  onTouchStart={() => this.controlUpdate(1<<4) }
                  onTouchEnd={() => this.controlUpdate(-1<<4)}
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
              <Image style={styles.image} source={require('../ControllerApp/assets/rassor.png')}/>
              <View style={{ flexDirection: 'row' }}>
                <View style={{ flexDirection: 'row' }}>
                  <View 
                  onTouchStart={() => this.controlUpdate(1<<3) }
                  onTouchEnd={() => this.controlUpdate(-1<<3)}
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
                  onTouchStart={() => this.controlUpdate(1<<2) }
                  onTouchEnd={() => this.controlUpdate(-1<<2)}
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
                  onTouchStart={() => this.controlUpdate(1<<0) }
                  onTouchEnd={() => this.controlUpdate(-1<<0)}
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
                  onTouchStart={() => this.controlUpdate(1<<1) }
                  onTouchEnd={() => this.controlUpdate(-1<<1)}
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

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#5D6061',
    alignItems: 'center',
    justifyContent: 'center',
  },

  headerContainer: {
    flex: 1,
    flexDirection: 'row',
    marginTop: 10,
    marginHorizontal: 10,
    elevation: 3,
    backgroundColor: '#2e3030',
    borderRadius: 10,
    padding: 10,
    justifyContent: 'center',
    alignItems: 'center'
  },

  buttonLayoutContainer: {
    flex: 8,
    flexDirection: 'row',
    marginVertical: 10,
  },

  text: {
    fontFamily: 'NASA',
    flex: 4,
    fontSize: 25,
    color: '#fff',
    textAlign: 'center',
    textAlignVertical: 'center',
  },

  image: {
    flex: 1,
    width: null,
    height: null,
    resizeMode: 'contain',
    paddingVertical:20,
  },

  modalViewContainer: {
    borderRadius: 25,
    backgroundColor: '#5D6061',
  },

  modalButton: {
    flex: 1,
    backgroundColor: '#767676',
    borderRadius: 100,
    marginHorizontal: 15,
    justifyContent: 'center',
    alignItems: 'center',
    height: 100,
    elevation: 5,
  },

  upAndDownDPad: {
    flex: 1, 
    backgroundColor: '#3a3d3d', 
    borderRadius: 10, 
    margin: 10, 
    elevation: 5, 
    justifyContent: 'center', 
    alignItems: 'center'
  },

  dPadLeft: {
    flex: 1, 
    backgroundColor: '#3a3d3d', 
    borderRadius: 10, 
    marginHorizontal: 10, 
    elevation: 5, 
    justifyContent: 'center', 
    alignItems: 'center'
  },

  dPadRight: {
    flex: 1, 
    backgroundColor: '#3a3d3d', 
    borderRadius: 10,
    marginRight: 10, 
    elevation: 5, 
    justifyContent: 'center', 
    alignItems: 'center'
  },

  drumFunctionContainer: {
    flex: 6, 
    justifyContent: 'center', 
    marginHorizontal: 10,
    padding:10, 
    borderRadius: 10, 
    elevation: 3, 
    backgroundColor: '#2e3030'
  },

  ipInputBox: {
    height: 80, 
    fontSize: 65, 
    backgroundColor:'#2e3030', 
    borderColor: 'gray', 
    borderWidth: 1 , 
    color: '#fff', 
    textAlign: 'center',
    textAlignVertical: 'center', 
    fontFamily: 'NASA',
  }
});

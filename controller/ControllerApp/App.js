import React from 'react';
import { StyleSheet, Text, View, TouchableHighlight, TouchableOpacity, Image } from 'react-native';
import { FontAwesome } from '@expo/vector-icons';
import Modal from "react-native-modal";

export default class App extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      command: "",
      modalVisible: false,
    };
    this.handleSubmit = this.handleSubmit.bind(this);
  }

  setModalVisible(visible) {
    this.setState({ modalVisible: visible });
  }

  async handleSubmit() {
    /*
        const command = "/api?command=" + this.state.command
        const response = await fetch(command, {method: 'POST'});
    
        if (!response.ok) {
          alert("Server Down");
          throw Error(response.statusText);
        }
          
        const data = await response.json();
    
        if (data == true) {
          ToastAndroid.showWithGravity(
            'Command Processed Successfully',
            ToastAndroid.SHORT,
            ToastAndroid.CENTER
          );
        } else {
          ToastAndroid.showWithGravity(
            'Command Failed',
            ToastAndroid.SHORT,
            ToastAndroid.CENTER
          );
        }
    */
  };

  render() {
    return (
      <View style={styles.container}>

        <Modal
          style={styles.modalViewContainer}
          isVisible={this.state.modalVisible}
          onSwipe={() => this.setModalVisible(!this.state.modalVisible)}
          swipeDirection='down'
          onRequestClose={() => {
            Alert.alert('Modal has been closed.');
          }}>
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

        <View style={styles.headerContainer}>
          <View style={{flexDirection: 'row', width: '97%' }}>
            <TouchableOpacity style={{ flex: 1 }}>
              <FontAwesome
                name="info-circle"
                size={32}
                color='#fff'
              />
            </TouchableOpacity>
            <Text style={styles.text}>EZ-RASSOR Controller</Text>
            <TouchableOpacity style={{ flex: 1 }} onPress={() => { this.setModalVisible(true); }}>
              <FontAwesome
                style={{ position: 'absolute', right: 0 }}
                name="eercast"
                size={30}
                color='#fff'
              />
            </TouchableOpacity>
          </View>
        </View>

        <View style={styles.buttonLayoutContainer}>
          <View style={{ flex: 1, justifyContent: 'space-around', marginHorizontal: 10 }}>
            <TouchableOpacity onPress={() => this.handleSubmit()}>
              <FontAwesome
                name="arrow-circle-left"
                size={50}
                color='#fff'
              />
            </TouchableOpacity>
          </View>
          <View style={{ flex: 1, justifyContent: 'space-around', marginHorizontal: 10 }}>
            <TouchableOpacity>
              <FontAwesome
                name="arrow-circle-up"
                size={50}
                color='#fff'
              />
            </TouchableOpacity>
            <TouchableOpacity>
              <FontAwesome
                name="arrow-circle-down"
                size={50}
                color='#fff'
              />
            </TouchableOpacity>
          </View>
          <View style={{ flex: 1, justifyContent: 'space-around', marginHorizontal: 10 }}>
            <TouchableOpacity>
              <FontAwesome
                name="arrow-circle-right"
                size={50}
                color='#fff'
              />
            </TouchableOpacity>
          </View>
          <View style={{ flex: 6, justifyContent: 'center', marginHorizontal: 20 }}>
            <View style= {{ flex: 1}}/>
            <View style= {{ flex: 8}}>
              <View style={{ flexDirection: 'row' }}>
                <View style={{ flexDirection: 'row' }}>
                  <TouchableOpacity onPress={() => this.handleSubmit()}>
                    <FontAwesome
                      name="arrow-circle-up"
                      size={50}
                      color='#fff'
                    />
                  </TouchableOpacity>
                  <TouchableOpacity style={{ marginHorizontal: 15 }} onPress={() => this.handleSubmit()}>
                    <FontAwesome
                      name="arrow-circle-down"
                      size={50}
                      color='#fff'
                    />
                  </TouchableOpacity>
                </View>
                <View style={{ flexDirection: 'row', position: 'absolute', right: 0 }}>
                  <TouchableOpacity style={{ marginHorizontal: 15 }} onPress={() => this.handleSubmit()}>
                    <FontAwesome
                      name="arrow-circle-up"
                      size={50}
                      color='#fff'
                    />
                  </TouchableOpacity>
                  <TouchableOpacity onPress={() => this.handleSubmit()}>
                    <FontAwesome
                      name="arrow-circle-down"
                      size={50}
                      color='#fff'
                    />
                  </TouchableOpacity>
                </View>
              </View>
              <Image
                style={styles.image}
                source={require('../ControllerApp/assets/rassor3.png')}
              />
              <View style={{ flexDirection: 'row' }}>
                <View style={{ flexDirection: 'row' }}>
                  <TouchableOpacity onPress={() => this.handleSubmit()}>
                    <FontAwesome
                      name="rotate-left"
                      size={50}
                      color='#fff'
                    />
                  </TouchableOpacity>
                  <TouchableOpacity style={{ marginHorizontal: 15 }} onPress={() => this.handleSubmit()}>
                    <FontAwesome
                      name="rotate-right"
                      size={50}
                      color='#fff'
                    />
                  </TouchableOpacity>
                </View>
                <View style={{ flexDirection: 'row', position: 'absolute', right: 0 }}>
                  <TouchableOpacity style={{ marginHorizontal: 15 }} onPress={() => this.handleSubmit()}>
                    <FontAwesome
                      name="rotate-left"
                      size={50}
                      color='#fff'
                    />
                  </TouchableOpacity>
                  <TouchableOpacity onPress={() => this.handleSubmit()}>
                    <FontAwesome
                      name="rotate-right"
                      size={50}
                      color='#fff'
                    />
                  </TouchableOpacity>
                </View>
              </View>
            </View>
            <View style= {{ flex: 1}}/>
          </View>
        </View>
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
    marginTop: 40,
    marginBottom: 10,
  },

  buttonLayoutContainer: {
    flex: 11,
    flexDirection: 'row',
  },

  text: {
    flex: 10,
    fontSize: 25,
    fontWeight: 'bold',
    color: '#fff',
    textAlign: 'center'
  },

  button: {

  },

  image: {
    flex: 1,
    width: null,
    height: null,
    resizeMode: 'contain',
  },

  modalViewContainer: {
    flex: 1,
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
    height: 100
  }
});

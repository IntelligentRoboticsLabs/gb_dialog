# GB-dialog
**GB-dialog simplifies the task of developing actions or behaviors related to dialogue.**

GB-dialog contains the library DialogInterface from which we will inherit to develop our dialogue actions. Each action would be specific to an intent ([Dialogflow concepts](https://dialogflow.com/docs)). The library offers methods to do speech-to-text tasks through [Google Speech sevice](https://cloud.google.com/speech-to-text/) and methods to do Natural Language Processing tasks through [Dialogflow](https://dialogflow.com/), using the ROS package dialogflow_ros ([official package](https://wiki.ros.org/dialogflow_ros),  [our custom dialogflow_ros](https://github.com/jginesclavero/dialogflow_ros)). The library also offers a method to do text-to-speech through the package [sound_play](https://wiki.ros.org/sound_play).

## Installing

### Install requirements
If you don't have vcs tool, install it with ```sudo apt-get install python3-vcstool wget```  


### Clone dialog packages

First of all, clone all the relevant pacakges for  dialog. Create a folder `dialog` and clone all required packages using the tool `vcs` from  `vcstool`:

```
roscd
cd ../src
mkdir dialog
cd dialog
wget https://raw.githubusercontent.com/IntelligentRoboticsLabs/gb_dialog/master/gb_dialog.repos
vcs import < gb_dialog.repos
```


### Setup

Original instructions are in this [README](https://github.com/jginesclavero/dialogflow_ros/tree/master/dialogflow_ros). Use it as reference:


#### Install the requirements for dialogflow_ros:

```
sudo apt-get install portaudio19-dev ros-noetic-rosbridge-server
roscd dialogflow_ros
pip install -r requirements.txt
```

#### Google Cloud and DialogFlow Setup

1. Go to [Google Cloud Console](https://console.cloud.google.com/).
2. Create a new project.
3. Go to [DialogFlow Console](https://dialogflow.cloud.google.com/) and create a New Agent.
4. Select the project and click on Create button.
5. Go to [Google Cloud Console](https://console.cloud.google.com/) again.
6. Following the [Kick Setup](https://cloud.google.com/dialogflow/es/docs/quick/setup):
  1. Enable API
  2. Create Service Account and download private keys as JSON
7. Download the JSON File. Rename and move it t your HOME as ~/df_api.json
8. Edit `dialogflow_ros/config/param.yaml` and write down your project id. You can find it in the [DialogFlow Console](https://dialogflow.cloud.google.com/), clicking in the gear icon.

## Use

Below is an example of using the GB-dialog library ([example file](https://github.com/IntelligentRoboticsLabs/gb_dialog/blob/master/gb_dialog/src/example/exampleDF.cpp)).
First of all we define our new class that inherits from DialogInterface.
Then, I must use the function ```registerCallback``` to define the handler of the DialogFlow response. You can indicate a specific intent on each callback to receive the DialogFlow responses in differents callbacks, or not to receive all results from DialogFlow in the same callback, up to you!

```
class ExampleDF: public DialogInterface
{
  public:
    ExampleDF(): nh_()
    {
      this->registerCallback(std::bind(&ExampleDF::noIntentCB, this, ph::_1));
      this->registerCallback(
        std::bind(&ExampleDF::welcomeIntentCB, this, ph::_1),
        "Default Welcome Intent");
    }
...
};

```
When we have our class instantiated we can use the methods *speak* or *listen*. Both methods are **syncronous**

```
gb_dialog::DialogInterfaceTest di();
di.speak("Hello world!")
```
```
gb_dialog::DialogInterfaceTest di();
di.listen();
```

## Tests
Compile and execute the test

```
roslaunch gb_dialog gb_dialog_services_soundplay.launch
rosrun gb_dialog example_df_node
```

And talk :)

### Creating a virtualenv for install Python modules
**If you have dependencies issues when you installed the above requirements**  
Create a virtualenv ```virtualenv venv --system-site-packages```  
Activate the virtual env ``` source venv/bin/activate ```  
Install the dependencies ``` pip install -r [__gb_dialog_Path__]/dialogflow_ros/dialogflow_ros/requirements.txt ```  

**Remind activate the virtualenv in each shell where you want use dialogflow_ros**

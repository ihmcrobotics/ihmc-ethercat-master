plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.3"
   id("us.ihmc.ihmc-ci") version "7.5"
   id("us.ihmc.ihmc-cd") version "1.21"
}

ihmc {
   group = "us.ihmc"
   version = "0.12.0"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-ethercat-master"
   openSource = true

   configureDependencyResolution()
   resourceDirectory("main", "swig")
   configurePublications()
}

app.entrypoint("SlaveInfo", "us.ihmc.etherCAT.master.SlaveInfo")

mainDependencies {
   api("us.ihmc:soem:1.4.0-ihmc1")
   api("us.ihmc:soem-platform-linux:1.4.0-ihmc1")
   api("us.ihmc:ihmc-native-library-loader:1.3.1")
   api("us.ihmc:ihmc-realtime:1.4.0")
}


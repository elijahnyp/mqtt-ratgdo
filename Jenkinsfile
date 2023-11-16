pipeline {
  agent {
    dockerfile {
      filename 'esp_idf_docker_builder/Dockerfile'
      args '--dns 10.0.1.44 --dns 10.0.1.43'
    }

  }
  stages {
    stage('build') {
      steps {
        sh '''export IDF_PATH=/esp-idf;
. /esp-idf/export.sh;
git submodule update --init --recursive
chmod +x fullbuild.sh
chmod +x build.sh
./fullbuild.sh'''
        sh '''cd out;
ls -lh
'''
      }
    }

    stage('test'){
      steps{
        echo 'Tests go here'
      }
    }

    stage('push'){
      steps{
        withCredentials([
          string(credentialsId: 'dd4a0bba-e788-4a6c-8b40-ea001648dcdd', variable: 'CLIENT_SECRET'),
          string(credentialsId: 'f14a5cbb-1220-4426-8710-32585989abc3', variable: 'REFRESH_TOKEN')]) {
           sh 'python3 publish.py'
        }
      }
    }
  }
  post {
    always {
      archiveArtifacts artifacts: 'out/*.bin', onlyIfSuccessful: true
      none{
        sh 'docker system prune -f --volumes'
      }
    }
  }
}
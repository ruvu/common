<template>
  <div class="card">
    <div class="card-header">
      Text to speech
    </div>
    <topic-selector :initialTopicName="topicName" @updated="topicName = $event"></topic-selector>
    <div class="card-block">
      <string-publisher :topicName="topicName" :historySize="historySize"></string-publisher>
      <b-input-group>
        <b-input-group-addon>History size</b-input-group-addon>
        <b-form-input v-model="historySize"
                      type="number"></b-form-input>
      </b-input-group>
    </div>
    <div class="card-footer text-muted">
      Publishing to {{topicName}}
    </div>
  </div>
</template>

<script>
import TopicSelector from '@/components/ros/TopicSelector'
import StringPublisher from '@/components/ros/StringPublisher'
import localStorageUtil from '@/util/localStorage'

export default {
  components: {
    TopicSelector,
    StringPublisher
  },
  data () {
    return {
      topicName: localStorageUtil.getLocalStorageWithDefault('TextToSpeech.topicName', 'cmd_vel'),
      historySize: 1 // localStorageUtil.getLocalStorageWithDefault('TextToSpeech.historySize', parseInt(5))
    }
  },
  watch: {
    topicName () {
      localStorage.setItem('TextToSpeech.topicName', this.topicName)
    },
    historySize () {
      this.historySize = parseInt(this.historySize)
      localStorage.setItem('TextToSpeech.historySize', this.historySize)
    }
  }
}
</script>

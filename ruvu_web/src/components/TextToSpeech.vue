<template>
  <div class="card">
    <div class="card-header">
      Text to speech
    </div>
    <topic-selector :initial="topicName" @updated="topicName = $event"></topic-selector>
    <div class="card-block">
      TODO
    </div>
    <div class="card-footer text-muted">
      Publishing to {{topicName}}
    </div>
  </div>
</template>

<script>
import TopicSelector from '@/components/ros/TopicSelector'
import localStorageUtil from '@/util/localStorage'

export default {
  components: {
    TopicSelector
  },
  data () {
    return {
      topicName: localStorageUtil.getLocalStorageWithDefault('TextToSpeech.topicName', 'cmd_vel')
    }
  },
  watch: {
    topicName () {
      localStorage.setItem('TextToSpeech.topicName', this.topicName)
    }
  }
}
</script>

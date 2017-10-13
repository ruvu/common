<template>
  <div class="card">
    <div class="card-header">
      HMI cast client
    </div>
    <action-selector :initialActionName="actionName" @updated="actionName = $event" messageType="hmi_msgs/CastAction"></action-selector>
    <div class="card-block">
      <hmi-cast-action-client :actionName="actionName" :historySize="historySize" :ros="ros"></hmi-cast-action-client>
      <b-input-group>
        <b-input-group-addon>History size</b-input-group-addon>
        <b-form-input v-model="historySize"
                      type="number"></b-form-input>
      </b-input-group>
    </div>
    <div class="card-footer text-muted">
      Action {{actionName}}
    </div>
  </div>
</template>

<script>
import ActionSelector from '@/components/ros/ActionSelector'
import HmiCastActionClient from '@/components/ros/HmiCastActionClient'
import localStorageUtil from '@/util/localStorage'
import ros from '@/services/ros'

export default {
  components: {
    ActionSelector,
    HmiCastActionClient
  },
  data () {
    return {
      ros: ros,
      actionName: localStorageUtil.getLocalStorageWithDefault('HmiCast.actionName', 'cast_action'),
      historySize: 1 // localStorageUtil.getLocalStorageWithDefault('TextToSpeech.historySize', parseInt(5))
    }
  },
  watch: {
    actionName () {
      localStorage.setItem('HmiCast.actionName', this.actionName)
      console.log('setting')
    },
    historySize () {
      this.historySize = parseInt(this.historySize)
      localStorage.setItem('HmiCast.historySize', this.historySize)
    }
  }
}
</script>

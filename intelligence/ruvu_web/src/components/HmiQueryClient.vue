<template>
  <div class="card">
    <div class="card-header">
      HMI query client
    </div>
    <action-selector :initialActionName="actionName" @updated="actionName = $event" messageType="hmi_msgs/QueryAction"></action-selector>
    <div class="card-block">
      <hmi-query-action-client :actionName="actionName" :ros="ros"></hmi-query-action-client>
    </div>
    <div class="card-footer text-muted">
      Action {{actionName}}
    </div>
  </div>
</template>

<script>
import ActionSelector from '@/components/ros/ActionSelector'
import HmiQueryActionClient from '@/components/ros/HmiQueryActionClient'
import localStorageUtil from '@/util/localStorage'
import ros from '@/services/ros'

export default {
  components: {
    ActionSelector,
    HmiQueryActionClient
  },
  data () {
    return {
      ros: ros,
      actionName: localStorageUtil.getLocalStorageWithDefault('HmiQuery.actionName', 'query_action')
    }
  },
  watch: {
    actionName () {
      localStorage.setItem('HmiQuery.actionName', this.actionName)
    }
  }
}
</script>

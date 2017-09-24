import Vue from 'vue'
import Router from 'vue-router'

import TwistTeleop from '@/components/TwistTeleop'
import TextToSpeech from '@/components/TextToSpeech'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      name: 'TwistTeleop',
      component: TwistTeleop
    },
    {
      path: '/text_to_speech',
      name: 'TextToSpeech',
      component: TextToSpeech
    }
  ]
})

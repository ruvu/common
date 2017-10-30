import Vue from 'vue'
import Router from 'vue-router'

import TwistTeleop from '@/components/TwistTeleop'
import StringPublisher from '@/components/StringPublisher'
import HmiCastClient from '@/components/HmiCastClient'
import HmiQueryClient from '@/components/HmiQueryClient'
import HmiServer from '@/components/HmiServer'
import NavigationViewer from '@/components/NavigationViewer'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      name: 'TwistTeleop',
      component: TwistTeleop
    },
    {
      path: '/string_publisher',
      name: 'StringPublisher',
      component: StringPublisher
    },
    {
      path: '/hmi_cast_client',
      name: 'HmiCastClient',
      component: HmiCastClient
    },
    {
      path: '/hmi_query_client',
      name: 'HmiQueryClient',
      component: HmiQueryClient
    },
    {
      path: '/hmi_server',
      name: 'HmiServer',
      component: HmiServer
    },
    {
      path: '/navigation_viewer',
      name: 'NavigationViewer',
      component: NavigationViewer
    }
  ]
})

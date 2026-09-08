import DefaultTheme from 'vitepress/theme'
import { h } from 'vue'
import ApiOutline from './ApiOutline.vue'
import './style.css'

export default {
  extends: DefaultTheme,
  Layout: () => h(DefaultTheme.Layout, null, {
    'aside-outline-before': () => h(ApiOutline),
  }),
}

import DefaultTheme from 'vitepress/theme'
import { h } from 'vue'
import SectionOutline from './SectionOutline.vue'
import './style.css'

export default {
  extends: DefaultTheme,
  Layout: () => h(DefaultTheme.Layout, null, {
    'aside-outline-before': () => h(SectionOutline),
  }),
}

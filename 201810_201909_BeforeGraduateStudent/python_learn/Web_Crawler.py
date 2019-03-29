#encoding: utf-8
# 简单的爬虫技术
# https://blog.csdn.net/csqazwsxedc/article/details/68498842
import requests
from lxml import html
url='https://movie.douban.com/' #需要爬数据的网址
page=requests.Session().get(url) 
tree=html.fromstring(page.text) 
# result=tree.xpath('//li[@class="title"]//a/text()') #获取需要的数据
result=tree.xpath('//li[@class="stitle"]//a/text()') #获取需要的数据
print "len(result) =", len(result)
for i in range(0, len(result)):
	print "result[i] = ",result[i]
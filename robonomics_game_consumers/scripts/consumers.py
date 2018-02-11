import httplib2
from re import findall

import apiclient
from oauth2client import client
from oauth2client import tools
from oauth2client.service_account import ServiceAccountCredentials


def download_ask_params(creds_file, spreadsheetId, rangeName):
    def download_sheet(creds_file, spreadsheetid, rangename):
        def get_values(service, spreadsheetId, rangeName):
            result = service.spreadsheets().values().get(
                spreadsheetId=spreadsheetId,
                range=rangeName,
                valueRenderOption='FORMULA'
                ).execute()
            return result.get('values')

        def get_service(creds_file):
            credentials = ServiceAccountCredentials.from_json_keyfile_name(creds_file,
                                    ['https://www.googleapis.com/auth/spreadsheets',
                                    'https://www.googleapis.com/auth/drive'])
            httpAuth = credentials.authorize(httplib2.Http())
            service = apiclient.discovery.build('sheets', 'v4', http = httpAuth)
            http = credentials.authorize(httplib2.Http())

            discoveryUrl = ('https://sheets.googleapis.com/$discovery/rest?'
                            'version=v4')
            service = apiclient.discovery.build('sheets', 'v4', http=http,
                                                discoveryServiceUrl=discoveryUrl)
            return service
        
        content = get_values(get_service(creds_file), spreadsheetId, rangeName)
        return content

    def extract_chart(content, topleft_content='Price', cols_num=17):
        row_chart = 0
        for row_number, row_content in enumerate(content):
            if topleft_content in row_content:
                row_chart = row_number
                break
        chart = content[row_chart:row_chart + cols_num]
        return chart

    ## @param max_price_line expected format:
    ##  max_price_line[0] - max price, int
    ##  max_price_line[1] - fee, int
    ##  max_price_line[2] - yellow chunks ask, formula
    ##  max_price_line[7] - green chunks ask, formula
    ##  max_price_line[12] - blue chunks ask, formula
    ##  max_price_line[17] - purple chunks ask, formula
    ## @return dict with parameters for robonomics_market/AsksGenerator message
    ##  {a: 10, k: 1, market: '', objective: '', fee: 1, price_range: 50}
    def extract_ask_params(max_price_line):
        def extract_ak(formula):
            nums = findall( r"[+-]?\d+(?:\.\d+)?", formula.replace(',', '.').replace(' ', '') )
            if len(nums) == 3: # get first two as a and k
                a = nums[0]
                k = nums[1]
            elif len(nums) == 2: # get first as k
                a = 0
                k = nums[0]
            elif len(nums) == 1: # get first as a
                a = nums[0]
                k = 0
            return {'a': float(a), 'k': float(k)}

        price_range = max_price_line[0]
        fee = max_price_line[1]
        ask = dict()
        ask['yellow'] = extract_ak(max_price_line[2])
        ask['green'] = extract_ak(max_price_line[7])
        ask['blue'] = extract_ak(max_price_line[12])
        ask['purple'] = extract_ak(max_price_line[17])
        for k, v in ask.items():
            v.update({'fee': fee, 'price_range': price_range})
        return ask

    content = download_sheet(creds_file, spreadsheetId, rangeName)
    chart = extract_chart(content)
    return extract_ask_params(chart[-1])

import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
import warnings
from langchain.prompts import PromptTemplate


class ExtractKeyword:
    def __init__(self):
        load_dotenv(dotenv_path=".env")
        openai_api_key = os.getenv("OPENAI_API_KEY")
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.0, openai_api_key=openai_api_key
        )
        prompt_content = """
            당신은 사용자의 음성 명령을 듣고 3x3 타일 배치 패턴을 결정해야 합니다.

            <타일 타입>
            - 1 = 흰색 타일
            - 2 = 검정 타일

            <3x3 배치 순서>
            - 왼쪽에서 오른쪽, 위에서 아래 순서로 1~9번
            - 예시:
              1 2 3
              4 5 6
              7 8 9

            <패턴 규칙>
            - "흰색 체크무늬" 또는 "흰검 체크무늬"
              → 흰색 시작 체크무늬: 1 2 1 2 1 2 1 2 1

            - "검정 체크무늬" 또는 "검흰 체크무늬"
              → 검정 시작 체크무늬: 2 1 2 1 2 1 2 1 2

            - "전부 흰색" 또는 "흰색으로 깔아" 또는 "흰색 타일" 또는 "흰 타일" 또는 "하얀 타일" 또는 "흰색만"
              → 1 1 1 1 1 1 1 1 1

            - "전부 검정" 또는 "검정으로 깔아" 또는 "검정 타일" 또는 "검은 타일" 또는 "검정만" 또는 "까만 타일"
              → 2 2 2 2 2 2 2 2 2

            - "데코" 또는 "데코 타일" 또는 "데코타일" 또는 "포인트 타일" 또는 "특수 타일"
              → 2 2 2 2 2 2 2 2 2

            <특수 규칙>
            - 명확한 패턴이 없으면 흰색 체크무늬(1 2 1 2 1 2 1 2 1)를 기본값으로 사용
            - "체크무늬" 단독 언급 시 흰색 체크무늬로 처리

            <오인식 보정>
            - "검색" → "검정"으로 해석
            - "흰색" 대신 "힌색", "흰새", "흰색" 등 유사 발음 → "흰색"으로 해석
            - "체크무늬" 대신 "체크", "체크 무늬", "체크뮤니" 등 → "체크무늬"로 해석
            - 의미가 불분명하더라도 색상(흰/검)과 패턴(체크/전체) 힌트가 있으면 최대한 해석

            <출력 형식>
            - 반드시 9개의 숫자를 공백으로 구분하여 한 줄로만 출력
            - 다른 설명이나 문장은 절대 출력하지 말 것
            - 예시 출력: 1 2 1 2 1 2 1 2 1

            <예시>
            - 입력: "흰색 체크무늬로 깔아줘"
              출력: 1 2 1 2 1 2 1 2 1

            - 입력: "검정 체크무늬로 깔아"
              출력: 2 1 2 1 2 1 2 1 2

            - 입력: "전부 흰색으로 깔아"
              출력: 1 1 1 1 1 1 1 1 1

            - 입력: "검정으로 다 깔아줘"
              출력: 2 2 2 2 2 2 2 2 2

            - 입력: "체크무늬로 깔아"
              출력: 1 2 1 2 1 2 1 2 1

            - 입력: "데코타일로 깔아줘"
              출력: 2 2 2 2 2 2 2 2 2

            <사용자 입력>
            "{user_input}"
        """
        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content.strip()

        # 숫자 9개 파싱
        tokens = result.split()
        if len(tokens) != 9:
            warnings.warn(f"응답 형식이 올바르지 않습니다: {result}")
            return None

        try:
            layout = [int(t) for t in tokens]
        except ValueError:
            warnings.warn(f"숫자 변환 실패: {result}")
            return None

        # 유효한 타일 타입인지 확인 (1, 2만 허용)
        if not all(t in [1, 2] for t in layout):
            warnings.warn(f"유효하지 않은 타일 타입 포함: {layout}")
            return None

        print(f"llm's response(layout): {layout}")
        print(f"패턴 시각화:")
        for row in range(3):
            row_str = ""
            for col in range(3):
                tile = layout[row * 3 + col]
                row_str += "⬜" if tile == 1 else "⬛"
            print(f"  {row_str}")

        return layout


if __name__ == "__main__":
    extractor = ExtractKeyword()

    test_cases = [
        "흰색 체크무늬로 깔아줘",
        "검정 체크무늬로 깔아",
        "전부 흰색으로 깔아",
        "검정으로 다 깔아줘",
        "체크무늬로 깔아",
        "데코타일로 깔아줘",
    ]

    for test in test_cases:
        print(f"\n입력: {test}")
        result = extractor.extract_keyword(test)
        print(f"layout: {result}")
